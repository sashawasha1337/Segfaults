

import os
import sys
print(sys.executable)
from uuid import uuid4
import firebase_admin
from firebase_admin import credentials, firestore, storage
import urllib.parse


SERVICE_ACCOUNT_FILE="image-test-permissions.json"

script_dir = os.path.dirname(os.path.abspath(__file__))
cred_path = os.path.join(script_dir, SERVICE_ACCOUNT_FILE)

if not firebase_admin._apps:
    cred = credentials.Certificate(cred_path)
    firebase_admin.initialize_app(cred, {
        'storageBucket': 'segfaults-database.firebasestorage.app'
    })

db = firestore.client()
bucket = firebase_admin.storage.bucket()

imagePath = "goku.jpg"
blob = bucket.blob("testfolder/goku.jpg")

token = str(uuid4())
blob.metadata = {"firebaseStorageDownloadTokens": token}

blob.upload_from_filename(imagePath,content_type="image/jpeg")

print("File uploaded to Firebase Storage.")
fixedname=urllib.parse.quote(blob.name, safe='')
image_url = f"https://firebasestorage.googleapis.com/v0/b/{bucket.name}/o/{fixedname}?alt=media&token={token}"
print("Download URL:", image_url)