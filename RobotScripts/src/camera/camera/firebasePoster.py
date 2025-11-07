#export FIREB



import os
import json
from flask import Flask, request, jsonify
import requests

from google.oauth2 import service_account
from google.auth.transport.requests import Request, AuthorizedSession

PROJECT_ID = "segfaults-database"   
SERVICE_ACCOUNT_PATH = os.getenv("robot-service-account.json") ##change this to real path
FIRESTORE_COLLECTION = os.getenv("FIRESTORE_COLLECTION", "events")

SCOPES = ["https://www.googleapis.com/auth/datastore"]

with open(SERVICE_ACCOUNT_PATH, "r") as f:
    sa_info = json.load(f)
PROJECT_ID = sa_info["project_id"]

CREDENTIALS = service_account.Credentials.from_service_account_info(sa_info, scopes=SCOPES)

BASE_URL = "https://firestore.googleapis.com/v1"
DOCS_URL = "{}/projects/{}/databases/(default)/documents".format(BASE_URL, PROJECT_ID)

app = Flask(__name__)

ALLOWED_FIELDS = {
    "category",
    "location",
    "robotId",
    "time",
    "users",
    "image_url",
    "confidence",
}

def validate_payload(d):
    if not isinstance(d, dict):
        return False, "Payload must be a JSON object"
    missing = [k for k in ALLOWED_FIELDS if k not in d]
    if missing:
        return False, f"Missing fields: {', '.join(missing)}"
    return True, ""

def to_firestore_format(d):
    if val is None:
        return {"nullValue": None}
    if key == "confidence":
        try:
            return {"doubleValue": float(val)}
        except Exception:
            return {"stringValue": str(val)}
        if key == "users" and isinstance (val, list):
            return {"arrayValue": {"values": [{"stringValue": str(u)} for u in val]}}
        if key == "time:" and str(val).endswith("Z"):
            return {"timestampValue": str(val)} 
        return {"stringValue": str(val)}

def ingest():
    if not request.is_json:
        return jsonify({"error": "Invalid JSON"}), 400
    data = request.get_json(silent_=True) or {}
    valid, msg = validate_payload(data)
    if not valid:
        return jsonify({"error": msg}), 400
    filtered = {k: v for k, v in data.items() if k in ALLOWED_FIELDS}
    fields = {k: to_firestore_format(k, v) for k, v in filtered.items()}

    body = {fields}

    creds = CREDENTIALS
    if not creds.valid:
        creds.refresh(Request())
    authed = AuthorizedSession(creds)
                               
    url = "{}/{}".format(DOCS_URL, FIRESTORE_COLLECTION)
    resp = authed.post(url, json=body)
    if resp.status_code >= 300:
        return jsonify({"error": "Failed to write to Firestore", "details": resp.text}), 502

    out = resp.json()
    name = out.get("name", "")
    doc_id = name.split("/")[-1] if name else None
    return jsonify(ok=True, id=doc_id), 200

if __name__ == "__main__":
    port = int(os.getenv("PORT", "8080"))
    app.run(host="0.0.0.0", port=port)
