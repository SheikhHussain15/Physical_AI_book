import os
import cohere
import requests
import argparse
import logging
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from uuid import uuid4

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def load_environment():
    """Loads environment variables from a .env file."""
    load_dotenv()
    logging.info("Environment variables loaded.")

def get_cohere_client():
    """Initializes and returns the Cohere client."""
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        logging.error("COHERE_API_KEY not found in environment variables.")
        raise ValueError("COHERE_API_KEY not found in environment variables.")
    logging.info("Cohere client initialized.")
    return cohere.Client(cohere_api_key)

def get_qdrant_client():
    """Initializes and returns the Qdrant client."""
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    if not qdrant_url or not qdrant_api_key:
        logging.error("QDRANT_URL or QDRANT_API_KEY not found in environment variables.")
        raise ValueError("QDRANT_URL or QDRANT_API_KEY not found in environment variables.")
    logging.info("Qdrant client initialized.")
    return QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

def create_qdrant_collection(client: QdrantClient, collection_name: str, vector_size: int):
    """Creates a Qdrant collection if it doesn't exist."""
    try:
        client.get_collection(collection_name=collection_name)
        logging.info(f"Collection '{collection_name}' already exists.")
    except Exception as e:
        logging.info(f"Collection '{collection_name}' not found. Creating it...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
        logging.info(f"Collection '{collection_name}' created.")

def fetch_html(url: str) -> str | None:
    """Fetches the HTML content of a page."""
    try:
        response = requests.get(url)
        response.raise_for_status()
        logging.info(f"Successfully fetched HTML from {url}")
        return response.text
    except requests.RequestException as e:
        logging.error(f"Error fetching {url}: {e}")
        return None

def crawl_site(start_url: str) -> set[str]:
    """Crawls a website and returns a set of unique, same-domain URLs."""
    domain = urlparse(start_url).netloc
    urls_to_visit = {start_url}
    visited_urls = set()
    logging.info(f"Starting crawl from: {start_url}")

    while urls_to_visit:
        url = urls_to_visit.pop()
        if url in visited_urls:
            continue

        html = fetch_html(url)
        if not html:
            continue

        visited_urls.add(url)
        logging.info(f"Crawled: {url}")

        soup = BeautifulSoup(html, "html.parser")
        for link in soup.find_all("a", href=True):
            href = link["href"]
            full_url = urljoin(url, href)
            
            # Check if the link is on the same domain and not an anchor link
            if urlparse(full_url).netloc == domain and "#" not in full_url:
                if full_url not in visited_urls:
                    urls_to_visit.add(full_url)

    logging.info(f"Finished crawling. Found {len(visited_urls)} unique URLs.")
    return visited_urls

def extract_text_from_html(html: str) -> str:
    """Extracts the main text content from HTML."""
    soup = BeautifulSoup(html, "html.parser")
    
    # Attempt to find the main content in a Docusaurus article
    article = soup.find("article")
    if article:
        logging.info("Extracted text from <article> tag.")
        return article.get_text(separator="\n", strip=True)
        
    # Fallback to the body
    body = soup.find("body")
    if body:
        return body.get_text(separator="\n", strip=True)
        
    logging.warning("No main text content found in HTML.")
    return ""

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> list[str]:
    """Splits text into smaller chunks with a specified size and overlap."""
    if not text:
        logging.warning("No text to chunk.")
        return []
    
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - overlap
        if start >= len(text):
            break
            
    logging.info(f"Chunked text into {len(chunks)} chunks.")
    return chunks

def generate_embeddings(co_client: cohere.Client, texts: list[str]) -> list[list[float]]:
    """Generates embeddings for a list of texts using the Cohere client."""
    try:
        response = co_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="classification" 
        )
        logging.info(f"Generated {len(response.embeddings)} embeddings.")
        return response.embeddings
    except Exception as e:
        logging.error(f"Error generating embeddings: {e}")
        raise

def store_in_qdrant(q_client: QdrantClient, collection_name: str, chunks: list[str], embeddings: list[list[float]], url: str):
    """Stores chunks and their embeddings in the Qdrant collection."""
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        points.append(
            models.PointStruct(
                id=str(uuid4()), # Generate a unique ID for each point
                vector=embedding,
                payload={"text": chunk, "source_url": url}
            )
        )
    try:
        q_client.upsert(
            collection_name=collection_name,
            points=points
        )
        logging.info(f"Stored {len(points)} points in Qdrant collection '{collection_name}'.")
    except Exception as e:
        logging.error(f"Error storing points in Qdrant collection '{collection_name}': {e}")
        raise

def run_ingestion_pipeline(start_url: str, collection_name: str, vector_size: int):
    """Orchestrates the data ingestion pipeline."""
    try:
        load_environment()
        co_client = get_cohere_client()
        q_client = get_qdrant_client()
        create_qdrant_collection(q_client, collection_name, vector_size)

        crawled_urls = crawl_site(start_url)
        logging.info(f"Finished crawling. Found {len(crawled_urls)} URLs.")

        for url in crawled_urls:
            try:
                html_content = fetch_html(url)
                if html_content:
                    text_content = extract_text_from_html(html_content)
                    if text_content:
                        chunks = chunk_text(text_content)
                        if chunks:
                            embeddings = generate_embeddings(co_client, chunks)
                            store_in_qdrant(q_client, collection_name, chunks, embeddings, url)
                        else:
                            logging.warning(f"No chunks generated for {url}")
                    else:
                        logging.warning(f"No text extracted from {url}")
                else:
                    logging.warning(f"Could not fetch HTML from {url}")
            except Exception as e:
                logging.error(f"Error processing URL {url}: {e}")
                
    except Exception as e:
        logging.critical(f"An unhandled error occurred in the ingestion pipeline: {e}")
        raise

def search(query_text: str, co_client: cohere.Client, q_client: QdrantClient, collection_name: str, top_k: int = 3):
    """Searches the Qdrant collection for relevant documents."""
    load_environment() # Ensure environment is loaded for search
    query_embedding = generate_embeddings(co_client, [query_text])[0]
    
    logging.info(f"Searching Qdrant collection '{collection_name}' for query: '{query_text}'")
    search_result = q_client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True 
    )
    
    logging.info(f"Found {len(search_result)} search results.")
    return search_result


if __name__ == "__main__":
    COLLECTION_NAME = "technical_book"
    VECTOR_SIZE = 1024

    parser = argparse.ArgumentParser(description="RAG Ingestion and Search Pipeline for Book Website.")
    parser.add_argument("--ingest", action="store_true", help="Initiate the data ingestion pipeline. Requires --url.")
    parser.add_argument("--url", type=str, help="Specify the starting URL for the ingestion pipeline (e.g., 'https://www.example.com/docs'). Required with --ingest.")
    parser.add_argument("--search", type=str, help="Perform a semantic search in the Qdrant collection with the provided query string (e.g., 'what is ROS2?').")

    args = parser.parse_args()

    if args.ingest:
        if not args.url:
            parser.error("--url is required for ingestion.")
        logging.info(f"Starting ingestion pipeline for URL: {args.url}")
        run_ingestion_pipeline(args.url, COLLECTION_NAME, VECTOR_SIZE)
    elif args.search:
        load_environment()
        co_client = get_cohere_client()
        q_client = get_qdrant_client()
        search_results = search(args.search, co_client, q_client, COLLECTION_NAME)
        logging.info("\nSearch Results:")
        for hit in search_results:
            logging.info(f"  Score: {hit.score}")
            logging.info(f"  URL: {hit.payload['source_url']}")
            logging.info(f"  Text: {hit.payload['text'][:200]}...")
            logging.info("-" * 20)
    else:
        parser.print_help()