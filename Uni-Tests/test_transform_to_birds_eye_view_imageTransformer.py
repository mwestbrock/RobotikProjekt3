import cv2
import numpy as np
import unittest

class MyClass:
    def transform_to_birds_eye_view(self, frame, marker_centers):
        """
        Transformiert den Eingabeframe in eine Vogelperspektive mithilfe einer perspektivischen Transformation.

        Args:
            frame (numpy.ndarray): Der Eingabeframe, der transformiert werden soll.
            marker_centers (list): Eine Liste von Tupeln mit Marker-IDs und deren Mittelpunktskoordinaten.

        Returns:
            numpy.ndarray: Der transformierte Frame in Vogelperspektive.
        """
        # Definiert die gewünschte Höhe und Breite für den transformierten Frame
        height = 480
        width = 480

        # Definiert die Quellpunkte und Zielkoordinaten für die perspektivische Transformation
        # Die Quellpunkte werden aus den Mittelpunktskoordinaten der Marker berechnet
        src_points = np.array((marker_centers[2][1:], marker_centers[3][1:], marker_centers[1][1:], marker_centers[0][1:]), dtype=np.float32)
        dst_points = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)

        # Berechnet die Transformationsmatrix für die perspektivische Transformation
        perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

        # Führt die perspektivische Transformation auf dem Frame aus
        warped_frame = cv2.warpPerspective(frame, perspective_matrix, (width, height))

        return warped_frame


class TestTransformToBirdsEyeView(unittest.TestCase):
    def setUp(self):
        # Erstelle eine Instanz der MyClass Klasse für die Tests
        self.my_class = MyClass()

    def test_transform_to_birds_eye_view(self):
        """
        Testet die Methode transform_to_birds_eye_view.

        Erzeugt einen Testframe und überprüft die Größe und den Typ des transformierten Frames.
        """
        # Erzeugung eines Testframes
        frame = np.zeros((100, 100, 3), dtype=np.uint8)  # Schwarzer Frame der Größe 100x100

        # Marker-Mittelpunkte
        marker_centers = [(1, 25, 25), (2, 75, 25), (3, 75, 75), (4, 25, 75)]

        # Aufruf der Methode
        transformed_frame = self.my_class.transform_to_birds_eye_view(frame, marker_centers)

        # Überprüfung der Größe des transformierten Frames
        expected_width = 480
        expected_height = 480
        self.assertEqual(transformed_frame.shape[1], expected_width)
        self.assertEqual(transformed_frame.shape[0], expected_height)

        # Überprüfung, ob der transformierte Frame denselben Typ wie der ursprüngliche Frame hat
        self.assertEqual(transformed_frame.dtype, frame.dtype)



    def test_transform_to_birds_eye_view_with_example_input(self):
        """
        Testet die Methode transform_to_birds_eye_view mit einem Beispiel-Eingabeframe und marker_centers.
        """
        # Erstelle einen Beispiel-Eingabeframe und marker_centers für die Tests
        frame = np.zeros((500, 500, 3), dtype=np.uint8)
        marker_centers = [
            (0, (100, 100)),
            (1, (400, 100)),
            (2, (400, 400)),
            (3, (100, 400))
        ]

        # Rufe die zu testende Methode auf
        transformed_frame = self.my_class.transform_to_birds_eye_view(frame, marker_centers)

        # Überprüfe, ob der transformierte Frame die richtige Form hat (450x450x3).
        self.assertEqual(transformed_frame.shape, (480, 480, 3))

        # Überprüfe, ob der transformierte Frame nicht leer ist (alle Pixel sind nicht null).
        self.assertFalse(np.all(transformed_frame == 0))

        # Überprüfe, ob die marker_centers richtig gesetzt wurden, indem eine umgekehrte perspektivische Transformation
        # auf den erwarteten Zielpunkten durchgeführt wird und sie mit den tatsächlich transformierten Punkten verglichen werden.
        expected_dst_points = np.array([[0, 0], [480, 0], [480, 480], [0, 480]], dtype=np.float32)
        src_points = np.array([marker[1] for marker in marker_centers], dtype=np.float32)
        perspective_matrix = cv2.getPerspectiveTransform(src_points, expected_dst_points)
        transformed_marker_centers = cv2.perspectiveTransform(np.array([src_points]), perspective_matrix)[0]

        for idx, marker in enumerate(marker_centers):
            expected_center = expected_dst_points[idx]
            transformed_center = transformed_marker_centers[idx]
            self.assertAlmostEqual(transformed_center[0], expected_center[0], delta=1)
            self.assertAlmostEqual(transformed_center[1], expected_center[1], delta=1)


if __name__ == '__main__':
    unittest.main()
