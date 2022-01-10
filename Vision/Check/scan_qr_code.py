import cv2
import pyzbar.pyzbar as pyzbar

cap = cv2.VideoCapture(0)
def rescale_frame(frame, percent):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


while True :
    _, frame = cap.read()
    frame40 = rescale_frame(frame, percent=40)
    decode_QR = pyzbar.decode(frame40)
    for qrcode in decode_QR:
        (x,y,w,h) = qrcode.rect
        cv2.rectangle(frame40, (x,y),(x + w, y +w),(0,0,255), 2)
        cv2.putText(frame40, str(qrcode.data), (100, 100), cv2.FONT_HERSHEY_PLAIN,2,(255,0,0), 3)
    cv2.imshow("QR Code Scanner",frame40)
    if cv2.waitKey(1) & 0xFF == ord("q") :
        break 

#cap.release()
#cv2.destroyAllWindows()