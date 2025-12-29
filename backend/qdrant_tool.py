import os
import cohere
import qdrant_client
import json
from dotenv import load_dotenv

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

cohere_client = cohere.Client(COHERE_API_KEY)
qdrant_client = qdrant_client.QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

COLLECTION_NAME = "rag_embedding"
EMBEDDING_MODEL = "embed-english-v3.0"

def qdrant_retrieval_tool(query: str, collection_name: str = COLLECTION_NAME, top_k: int = 5) -> str:
    """
    Embeds the query using Cohere and searches Qdrant for relevant chunks.
    Returns a JSON string with context text and a list of source URLs.
    """
    try:
        query_embedding = cohere_client.embed(texts=[query], model=EMBEDDING_MODEL).embeddings[0]
    except cohere.CohereError as e:
        return f"Error embedding query: {e}"

    search_result = qdrant_client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=top_k,
        with_payload=True,
    )

    context_parts = []
    sources = set()
    for hit in search_result:
        text_content = hit.payload.get("text", "")
        source_url = hit.payload.get("source_url", "N/A")
        context_parts.append(text_content)
        if source_url != "N/A":
            sources.add(source_url)
    
    return json.dumps({
        "context": "\n".join(context_parts),
        "sources": list(sources)
    })
