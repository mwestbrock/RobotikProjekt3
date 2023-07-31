import unittest

def run_all_tests():

    """
    Führt alle Unit-Tests in unserem Projekt aus.

    Diese Funktion verwendet den TestLoader, um alle Tests in den
    Testdateien zu entdecken und auszuführen.
    """

    test_loader = unittest.TestLoader()
    test_suite = test_loader.discover(start_dir='robotik_projekt', pattern='test_*.py')

    # Ausführung der Tests mit einem TextTestRunner
    unittest.TextTestRunner().run(test_suite)

if __name__ == '__main__':
    run_all_tests()
