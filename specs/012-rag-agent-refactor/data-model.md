# Data Model: RAG Backend Refactor

This feature does not introduce new persistent data models. It interacts with the following transient data structures.

## Chat Request

-   **Description**: Represents a user's question to the chatbot.
-   **Fields**:
    -   `question` (string, required): The user's query.

## Chat Response

-   **Description**: Represents the chatbot's answer.
-   **Fields**:
    -   `answer` (string, required): The grounded answer to the user's question.
    -   `sources` (array of strings, required): A list of source URLs used to generate the answer.
