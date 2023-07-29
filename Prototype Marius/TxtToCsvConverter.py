import csv

# Header definieren
header = ['number_of_corners', 'area', 'radius']

# Daten aus der Textdatei lesen
with open('C:/Users/mariu/4. Semester/Projekt/Einhorn4.txt', 'r') as file:
    data = file.readlines()

# CSV-Datei schreiben
with open('C:/Users/mariu/4. Semester/Projekt/Einhorn4.csv', 'w', newline='') as file:
    writer = csv.writer(file)

    # Header schreiben
    writer.writerow(header)

    # Daten aus der Textdatei schreiben
    for line in data:
        # Entferne Zeilenumbruch am Ende jeder Zeile
        line = line.rstrip('\n')

        # Teile die Zeile anhand eines Trennzeichens, falls erforderlich
        # Beispiel: line = line.split(',')

        # Schreibe die Zeile als Datensatz in die CSV-Datei
        writer.writerow([line])
