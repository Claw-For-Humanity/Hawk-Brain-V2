import requests

# URL of the FastAPI endpoint
url = "http://localhost:4040/request_fs/"

# Path to the local image file
image_path = "./test_fhd.jpg"

# Open the image file in binary mode
with open(image_path, "rb") as image_file:
    # Create a dictionary with the image file
    files = {'file': image_file}
    
    # Send the POST request
    response = requests.post(url, files=files)

# Print the server's response
print(response.json())
