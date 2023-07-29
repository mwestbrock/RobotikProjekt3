file_path = 'C:/Users/mariu/4. Semester/Projekt/Einhorn4.txt'

# Öffne die Datei im Lese-Modus und lies den Inhalt
with open(file_path, 'r') as file:
    content = file.read()

# Ersetze die Kommas durch Komma und Leerzeichen
new_content = content.replace(',', ', ')

# Öffne die Datei im Schreib-Modus und schreibe den neuen Inhalt
with open(file_path, 'w') as file:
    file.write(new_content)
