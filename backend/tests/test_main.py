import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch
from main import app

client = TestClient(app)

def test_root_endpoint():
    """
    Test the root endpoint for basic connectivity.
    """
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "RAG Agent Service is running."}

@patch('main.run_agent_and_get_response')
def test_chat_endpoint_success(mock_run_agent):
    """
    Test the /chat endpoint with a mocked agent response.
    """
    # Configure the mock to return a specific answer and sources
    mock_run_agent.return_value = ("The capital of France is Paris.", ["http://example.com/france"])

    # Send a request to the /chat endpoint
    response = client.post("/chat", json={"question": "What is the capital of France?"})

    # Assertions
    assert response.status_code == 200
    json_response = response.json()
    assert json_response["answer"] == "The capital of France is Paris."
    assert json_response["sources"] == ["http://example.com/france"]
    mock_run_agent.assert_called_once_with("What is the capital of France?")

@patch('main.run_agent_and_get_response')
def test_chat_endpoint_agent_error(mock_run_agent):
    """
    Test the /chat endpoint when the agent raises an exception.
    """
    # Configure the mock to raise an exception
    mock_run_agent.side_effect = Exception("Agent failure")

    # Send a request to the /chat endpoint
    response = client.post("/chat", json={"question": "This will fail"})

    # Assertions
    assert response.status_code == 500
    assert response.json() == {"detail": "Agent failed to process the query: Agent failure"}