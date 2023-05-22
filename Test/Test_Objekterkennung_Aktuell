import unittest
import cv2
import numpy as np
import queue
import matplotlib.pyplot as plt
from Code import Greifpunkt as gp


class TestGreifpunkt(unittest.TestCase):
    # 1. Test, ob das Video richtig geladen wird
    def test_video_laden(self):
        video_path = 'C:/Users/Yusuf Sincar/Dropbox/PC/Desktop/EinhörnerUndKatzen/Videos/CatsUnicorns.mp4'
        cap = cv2.VideoCapture(video_path)

        if cap.isOpened():
            print(" Das Video wurde erfolgreich geladen.")
            self.assertTrue(True)
        else:
            print(" Fehler beim Öffnen des Videos")
            self.fail(" Fehler beim Öffnen des Videos")

        cap.release()

    # 2. Test, ob die Wartezeit richtig berechnet worden ist
    def test_wartezeit_berechnen(self):
        cap = cv2.VideoCapture('C:/Users/Yusuf Sincar/Dropbox/PC/Desktop/EinhörnerUndKatzen/Videos/CatsUnicorns.mp4')
        fps = cap.get(cv2.CAP_PROP_FPS)
        wait_time = int(1000 / fps)

        if wait_time == 33 and isinstance(wait_time, int) and fps != 0:
            print(" Die Wartezeit wurde erfolgreich berechnet.")
            self.assertTrue(True)
        else:
            print(" Ungültige Wartezeit")
            self.fail(" Ungültige Wartezeit")

        cap.release()

    # 3. Test, ob das Bild richtig skaliert wird
    def test_bild_skalieren(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        scale_percent = 50
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        if resized_frame.shape[1] == 320 and resized_frame.shape[0] == 240:
            print(" Das Bild wurde erfolgreich skaliert.")
            self.assertTrue(True)
        else:
            print(" Ungültige Dimensionen des skalierten Bildes")
            self.fail(" Ungültige Dimensionen des skalierten Bildes")

    # 4. Test, ob der Greifpunkt richtig ermittelt wird
    def test_greifpunkt_ermitteln(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        bounding_box = frame[100:200, 200:300]

        gripping_point = gp.grippingPoint(bounding_box)

        if isinstance(gripping_point, tuple) and len(gripping_point) == 2:
            print(" Der Greifpunkt wurde erfolgreich ermittelt.")
            self.assertTrue(True)
        else:
            print(" Der Greifpunkt sollte ein Tupel mit 2 Koordinaten sein")
            self.fail("Der Greifpunkt sollte ein Tupel mit 2 Koordinaten sein")


if __name__ == '__main__':
    # Führe die Tests aus
    test_suite = unittest.TestLoader().loadTestsFromTestCase(TestGreifpunkt)
    test_result = unittest.TextTestRunner(verbosity=2).run(test_suite)

    if test_result.wasSuccessful():
        print(" Alle Tests wurden erfolgreich durchgeführt.")
    else:
        print(" Es sind Fehler bei der Durchführung der Tests aufgetreten.")
