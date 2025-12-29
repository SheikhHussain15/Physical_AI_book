import os
import asyncio
from dotenv import load_dotenv
from openai import AsyncOpenAI
from openai.types.beta.threads import Message
from qdrant_tool import qdrant_retrieval_tool
from agents import OpenAIChatCompletionsModel, Runner, Agent

load_dotenv()

# Initialize OpenAI client
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
provider = AsyncOpenAI(
    api_key=OPENAI_API_KEY,
    base_url="https://openrouter.ai/api/v1",
)

# Define the Agent
instructions = """
You are a helpful RAG chatbot expert on a technical book. 
Use the 'qdrant_retrieval_tool' to find relevant information from the book before answering user questions. 
Always include the source URLs in your response if they are provided by the tool. 
If no context is found, state that you cannot answer the question.
"""

agent = Agent(
    model=OpenAIChatCompletionsModel(
        openai_client=provider,
        model="mistralai/devstral-2512:free"
    ),
    tools=[qdrant_retrieval_tool],
    instructions=instructions,
)

# Define the Runner
runner = Runner(agent=agent)

async def run_agent_and_get_response(user_query_text: str) -> tuple[str, list[str]]:
    """
    Processes a user query using the agent and runner, and returns the response.
    """
    thread = await runner.create_thread()
    await runner.add_message(thread_id=thread.id, content=user_query_text)
    
    # Stream the response
    response_stream = runner.run_stream(thread_id=thread.id)

    agent_answer = ""
    retrieved_sources = []

    async for chunk in response_stream:
        if chunk.event == "thread.message.delta":
            agent_answer += chunk.data.delta.content[0].text.value
        elif chunk.event == "thread.run.step" and chunk.data.step_details.type == "tool_calls":
            tool_call = chunk.data.step_details.tool_calls[0]
            if tool_call.function.name == "qdrant_retrieval_tool":
                # In a real scenario, you might want to inspect the tool output
                # For now, we assume the agent will include sources in its text response
                pass

    # For simplicity, we'll extract sources from the agent's text response.
    # A more robust solution is to parse the tool output from the run steps.
    # We will assume the agent has been instructed to format the sources in its response.
    
    # This is a placeholder for source extraction logic.
    # In a real implementation, you would parse the agent's response
    # to find the sources it cited.
    if "Sources:" in agent_answer:
        parts = agent_answer.split("Sources:")
        agent_answer = parts[0]
        sources_str = parts[1]
        # This is a naive way to parse sources. A real implementation should be more robust.
        retrieved_sources = [s.strip() for s in sources_str.split(',') if s.strip()]


    return agent_answer, retrieved_sources