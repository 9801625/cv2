import cv2
import numpy as np
IDs = {"01":"a","02":"b","03":"c","04":"d","05":"e","06":"f"}
cap = cv2.VideoCapture(0)
# initialize the cv2 QRCode detector
detector = cv2.QRCodeDetector()

while True:
    frame, img = cap.read()
    print(type(img))
    output_img = img.copy()
    data, bbox, _ = detector.detectAndDecode(img)
    
    if data:
        cv2.putText(output_img, data, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 5, cv2.LINE_AA)
        for i in range(4):
            cv2.line(output_img, tuple(bbox[i][0]), tuple(bbox[(i+1)%len(bbox)][0]), (0, 0, 255), 4)
    cv2.imshow("QRCODEscanner", output_img) 
    for i in data.split(","):
        if i.split(":")[0] in IDs.keys():
            print(f"{IDs[i.split(":")[0]]}が{int(i.split(":")[1])}個")     
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()