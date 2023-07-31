import cv2
import numpy as np
import unittest

class MyClass:
    def resize_frame(self, frame, scale_percent):

        """
        Verändert die Größe des Eingabeframes um einen gegebenen Prozentsatz.

        Args:
            frame (numpy.ndarray): Der Eingabeframe, der verändert werden soll.
            scale_percent (int): Der Prozentsatz zur Änderung der Größe.

        Returns:
            numpy.ndarray: Der veränderte Frame.
        """

        # Berechnet die gewünschte Breite und Höhe basierend auf dem gegebenen Prozentsatz
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)

        # Ändert die Größe des Frames unter Verwendung der berechneten Breite und Höhe
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        return resized_frame


class TestResizeFrame(unittest.TestCase):
    def test_resize_frame(self):

        """
        Testet die Methode resize_frame.

        Erzeugt einen Testframe und überprüft die Größe, den Typ und die Skalierung des veränderten Frames.
        """
        
        # Erzeugung eines Testframes
        frame = np.zeros((100, 100, 3), dtype=np.uint8)  # Schwarzer Frame der Größe 100x100

        # Aufruf der Methode mit einem Prozentsatz von 50
        my_class = MyClass()
        resized_frame = my_class.resize_frame(frame, 50)

        # Überprüfung der Größe des veränderten Frames
        expected_width = 50
        expected_height = 50
        self.assertEqual(resized_frame.shape[1], expected_width)
        self.assertEqual(resized_frame.shape[0], expected_height)

        # Überprüfung, ob der veränderte Frame denselben Typ wie der ursprüngliche Frame hat
        self.assertEqual(resized_frame.dtype, frame.dtype)

        # Überprüfung der Skalierung
        self.assertAlmostEqual(resized_frame.shape[1] / frame.shape[1], 0.5, delta=0.01)
        self.assertAlmostEqual(resized_frame.shape[0] / frame.shape[0], 0.5, delta=0.01)

if __name__ == '__main__':
    unittest.main()
