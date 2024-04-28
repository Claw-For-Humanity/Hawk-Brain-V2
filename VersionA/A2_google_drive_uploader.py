from googleapiclient.discovery import build
from google.oauth2 import service_account
import os

class initializer:
    SCOPES = ['https://www.googleapis.com/auth/drive'] # final

    PATH_SERVICE_ACCOUNT_FILE = None
    PATH_CAPTURED_IMAGES = None
    creds = None
    ID_PARENT_FOLDER = None


    def __init__(DEFAULT_PATH, account_id, destination_parent_id):
        '''example is
        (default_base_folder, "cfh-hawkeye-xxxxxx", "xxxxxx (end of the addy)")'''
        initializer.PATH_SERVICE_ACCOUNT_FILE = os.path.join(DEFAULT_PATH, 'data', f'{account_id}.json')
        initializer.PATH_CAPTURED_IMAGES = os.path.join(DEFAULT_PATH, 'capturedImages')
        initializer.ID_PARENT_FOLDER = destination_parent_id

        # define creds and authenticate
        initializer.creds = initializer.authenticate()

        
    
    def authenticate():
        creds = service_account.Credentials.from_service_account_file(initializer.PATH_SERVICE_ACCOUNT_FILE, scopes = initializer.SCOPES)
        return creds

class uploader:
    def photo(image_name, upload_name):
        '''make sure to include format of the file as well i.e.) .png '''
        if initializer.PATH_CAPTURED_IMAGES == None or initializer.PATH_SERVICE_ACCOUNT_FILE == None or initializer.creds == None:
            print("initialize first!")
            print(f"[LOG] : {initializer.PATH_CAPTURED_IMAGES}")
            print(f"[LOG] : {initializer.PATH_SERVICE_ACCOUNT_FILE}")
            print(f"[LOG] : {initializer.creds}")
            
            exit()
        
        service = build('drive', 'v3', credentials=initializer.creds)

        file_metadata = {
            'name':f"{upload_name}",
            'parents': [initializer.ID_PARENT_FOLDER]
        }

        file_path = os.path.join(initializer.PATH_CAPTURED_IMAGES, f'{image_name}')

        file = service.files().create(
            body=file_metadata,
            media_body=file_path
        ).execute()

        print('uploaded!')


# for debugging
# default = os.path.abspath('./')
# google_driver.__init__(default, "cfh-hawkeye-adb016b59179", "1NPciFwLIoW_ysdBg3ObeXRN4QvJBMXBl")
# google_driver.upload_photo("image.png", "test1.png")