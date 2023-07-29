#Support Vector Machine
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
import joblib

# Lade die CSV-Dateien
cats_data = pd.read_csv('C:/Users/mariu/4. Semester/Projekt/Katze4.csv', decimal=',')
unicorns_data = pd.read_csv('C:/Users/mariu/4. Semester/Projekt/Einhorn4.csv', decimal=',')

# Füge eine Spalte hinzu, um die Klassen (Labels) zu kennzeichnen
cats_data['label'] = '0'
unicorns_data['label'] = '1'

# Kombiniere die Daten aus beiden CSV-Dateien zu einem einzigen Datensatz
dataset = pd.concat([cats_data, unicorns_data], ignore_index=True)

# Trenne die Features (X) von den Labels (y)
X = dataset.drop('label', axis=1)
y = dataset['label']
#Setze Spaltennamen für das X dataframe
X.columns = ['number_of_corners', 'area', 'radius']


# Teile die Daten in Trainings- und Testsets auf
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Initialisiere den SVM-Klassifikator
svm = SVC()

# Trainiere den SVM-Klassifikator mit den Trainingsdaten
svm.fit(X_train, y_train)

from sklearn.metrics import classification_report

# Vorhersagen für die Testdaten
y_pred = svm.predict(X_test)

# Auswertung der Klassifikationsleistung
print(classification_report(y_test, y_pred))




# Speichern des Modells als Datei
joblib.dump(svm, 'C:/Users/mariu/4. Semester/Projekt/svm_v4.pkl.txt')


svm_loaded = joblib.load('C:/Users/mariu/4. Semester/Projekt/svm_v4.pkl.txt')

print(X.columns)
print(X.dtypes)
