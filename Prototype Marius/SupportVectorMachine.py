from sklearn import svm
from sklearn.model_selection import GridSearchCV
from sklearn.model_selection import train_test_split
import os
from skimage.io import imread
from skimage.transform import resize
import numpy as np
import pandas as pd
import joblib

def train_svm(image_data, image_labels):
    # Convert image data to flattened array
    flat_data = np.array(image_data)
    num_samples = flat_data.shape[0]
    flat_data = flat_data.reshape(num_samples, -1)
    target = np.array(image_labels)

    # Create DataFrame with flattened data and labels
    df = pd.DataFrame(flat_data)
    df['Target'] = target

    x = df.iloc[:, :-1].values
    y = df.iloc[:, -1].values

    # Define parameter grid for GridSearchCV
    param_grid = {'C': [0.1, 1, 10, 100], 'gamma': [0.0001, 0.001, 0.1, 1], 'kernel': ['rbf', 'poly']}
    svc = svm.SVC(probability=True)
    model = GridSearchCV(svc, param_grid)

    # Split data into training and testing sets
    x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.20, random_state=77, stratify=y)
    print('Data split successfully')

    # Train the SVM model
    model.fit(x_train, y_train)
    print('The model is trained well with the given images')

    # Perform predictions on the test set
    y_pred = model.predict(x_test)
    print("The predicted data is:")
    print(y_pred)
    print("The actual data is:")
    print(np.array(y_test))
    print("The accuracy of the model is:", model.score(x_test, y_test))

    # Return the trained model
    return model

"""def predict_with_svm(model, image):
    img_resize = resize(image, (150, 150, 3))
    flattened_img = img_resize.flatten()
    l = [flattened_img]

    # Perform prediction on the input image
    probability = model.predict_proba(l)
    for ind, val in enumerate(Categories):
        print(f'{val} = {probability[0][ind] * 100}%')
    #print("The predicted image is:", Categories[model.predict(l)[0]])
    cv2.putText(image, Categories[model.predict(l)[0]], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return"""

# Main program
Categories = ['cat', 'unicorn']
datadir = 'F:/mariu/Desktop/ProjektBilder/pics'
image_data = []
image_labels = []

for i in Categories:
    print('Loading... category:', i)
    path = os.path.join(datadir, i)
    for img in os.listdir(path):
        img_array = imread(os.path.join(path, img))
        img_resized = resize(img_array, (150, 150, 3))
        image_data.append(img_resized)
        image_labels.append(Categories.index(i))
    print('Loaded category:', i)

# Train the SVM model
svm_model = train_svm(image_data, image_labels)

joblib.dump(svm_model, 'F:/mariu/Desktop/ProjektBilder/pics/svm_model_V1.pkl')