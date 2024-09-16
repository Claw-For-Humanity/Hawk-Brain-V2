import sys, os, cv2, time
sys.path.append('../FastSAM-main')
import segeverything as sam

sys.path.append('../plugins/maxBoundingCropper')
from maxBoundingCropper import main as cropper

sys.path.append('../plugins/compressor')
from compressor import resize_with_padding as padder

sys.path.append('../plugins/tools')
import tools

img2test = '/Users/changbeankang/Claw_For_Humanity/HOS_II/Sample-Images/Original/5.jpg'
# img2test = '/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/test_fhd.jpg'
img2test = cv2.imread(img2test)

mode = 'x'
sam.initialize.init(mode, 0.4, 0.4)

er = sam.main.inference(img2test)
frame, objs = sam.main.annotate(er,is_plt = True, is_msk=False,frame = img2test)
# cv2.imwrite('/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/iphone/annotate.png',
#             frame)

boxes = tools.sam_to_coordinates(objs)
print(objs)

###### method 1: rescaled it in original ratio -> crop it 
img2test = padder(img2test,(1024,576))
# crop it into 2:1 ratio
img2test = cropper.crop_image_full_box(
                    image= img2test,
                    bounding_boxes= boxes
                    )

cv2.imwrite('/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/iphone/method1.png', img2test)


###### method 2: crop it into 2:1 ratio and pad it
orig_height, orig_width = img2test.shape[:2]
img2test = cropper.crop_image_full_box(
                    image= img2test,
                    bounding_boxes= boxes,
                    target_ratio= (int(orig_width), int(orig_width/2))
                    )
img2test = padder(img2test)
# cv2.imwrite('/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/test_fhd/method2.png', img2test)


###### method 3: just pad it
img2test = padder(img2test)
# cv2.imwrite('/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/test_fhd/method3.png', img2test)

###### method 4: just crop it
img2test = cropper.crop_image_full_box(
                    image= img2test,
                    bounding_boxes= boxes
                    )
# cv2.imwrite('/Users/changbeankang/Claw_For_Humanity/HOS_II/usage/test_fhd/method4.png', img2test)


# 1024 : 512 = 2:1
# 1920 : 1080 = 1920 : 960 