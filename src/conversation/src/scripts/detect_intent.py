import uuid
import os
from dotenv import load_dotenv
from google.cloud.dialogflowcx_v3beta1.services.agents import AgentsClient
from google.cloud.dialogflowcx_v3beta1.services.sessions import SessionsClient
from google.cloud.dialogflowcx_v3beta1.types import session
from google.api_core.client_options import ClientOptions
from google.protobuf.json_format import MessageToDict

def detect_intent_text(text, project_id, location_id, agent_id, session_id=None):
    """Returns the result of detect intent with texts as inputs.

    Using the same `session_id` between requests allows continuation
    of the conversation."""
        
    agent = f"projects/{project_id}/locations/{location_id}/agents/{agent_id}"
    if not session_id:
        session_id = uuid.uuid4()
    language_code = "en-us"

    session_path = f"{agent}/sessions/{session_id}"
    client_options = None
    agent_components = AgentsClient.parse_agent_path(agent)
    location_id = agent_components["location"]
    api_endpoint = f"{location_id}-dialogflow.googleapis.com:443"
    client_options = ClientOptions(api_endpoint=api_endpoint)
    session_client = SessionsClient(client_options=client_options)
    text_input = session.TextInput(text=text)
    query_input = session.QueryInput(text=text_input, language_code=language_code)
    request = session.DetectIntentRequest(
        session=session_path, query_input=query_input
    )
    response = session_client.detect_intent(request=request)

    print(f"Query text: {response.query_result.text}")
    response_messages = [
        " ".join(msg.text.text) for msg in response.query_result.response_messages
    ]

    parameters = MessageToDict(response._pb)
    
    print(f"Response text: {' '.join(response_messages)}\n")

    response_output = ' '.join(response_messages)

    return (response_output, parameters['queryResult']['parameters'])

if __name__ == "__main__":
    load_dotenv(override=True)
    PROJECT_ID = os.getenv("DFCX_PROJECT_ID")
    LOCATION_ID = os.getenv("DFCX_LOCATION_ID")
    AGENT_ID = os.getenv("DFCX_AGENT_ID")
    service_account_path = os.getenv("DFCX_SERVICE_ACCOUNT")
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = service_account_path

    detect_intent_text("Put the red box in the mat", project_id=PROJECT_ID, location_id=LOCATION_ID, agent_id=AGENT_ID, session_id="test-session")