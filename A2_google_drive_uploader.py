from googleapiclient.discovery import build
from google.oauth2 import service_account
import os

class google_driver:
    SCOPES = ['https://www.googleapis.com/auth/drive'] # final

    PATH_SERVICE_ACCOUNT_FILE = None
    PATH_CAPTURED_IMAGES = None
    creds = None
    ID_PARENT_FOLDER = None


    def __init__(DEFAULT_PATH, file_id, parent_id):
        google_driver.PATH_SERVICE_ACCOUNT_FILE = os.path.join(DEFAULT_PATH, 'data', f'{file_id}.json')
        google_driver.PATH_CAPTURED_IMAGES = os.path.join(DEFAULT_PATH, 'capturedImages')
        google_driver.ID_PARENT_FOLDER = parent_id

        # define creds and authenticate
        google_driver.creds = google_driver.authenticate()

        
    
    def authenticate():
        creds = service_account.Credentials.from_service_account_file(google_driver.PATH_SERVICE_ACCOUNT_FILE, scopes = google_driver.SCOPES)
        return creds


    def upload_photo(image_name, upload_name):
        '''make sure to include format of the file as well i.e.) .png '''
        if google_driver.PATH_CAPTURED_IMAGES == None or google_driver.PATH_SERVICE_ACCOUNT_FILE == None or google_driver.creds == None:
            print("initialize first!")
            print(f"[LOG] : {google_driver.PATH_CAPTURED_IMAGES}")
            print(f"[LOG] : {google_driver.PATH_SERVICE_ACCOUNT_FILE}")
            print(f"[LOG] : {google_driver.creds}")
            
            exit()
        
        service = build('drive', 'v3', credentials=google_driver.creds)

        file_metadata = {
            'name':f"{upload_name}",
            'parents': [google_driver.ID_PARENT_FOLDER]
        }

        file_path = os.path.join(google_driver.PATH_CAPTURED_IMAGES, f'{image_name}')

        file = service.files().create(
            body=file_metadata,
            media_body=file_path
        ).execute()

        print('uploaded!')

default = os.path.abspath('./')
google_driver.__init__(default, "cfh-hawkeye-adb016b59179", "1NPciFwLIoW_ysdBg3ObeXRN4QvJBMXBl")
google_driver.upload_photo("image.png", "test1.png")