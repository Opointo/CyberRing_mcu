# Description: This code is used to import the data and check the missing values, correlation, and build the model using Random Forest Classifier.
# '.csv' file columns: acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, row, pitch, yaw, qua_w, qua_x, qua_y, qua_z, target and timestamp.

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.ticker as mtick
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, roc_auc_score, confusion_matrix
import os

###数据整合###
dataframes = []     # Initialize an empty list to hold the dataframes
files = os.listdir('D:\\')  # Get a list of all files in the directory
for file in files:
    if file.endswith('.csv') and file.startswith('DATA_'):
        filepath = os.path.join('D:\\', file)
        df = pd.read_csv(filepath)
        dataframes.append(df)
# Concatenate all dataframes in the list
df = pd.concat(dataframes, ignore_index=True)
df.to_csv('dataset.csv', encoding='utf-8')
df.drop('timestamp', axis = 1, inplace = True)   # Drop the timestamp column

###数据检查###
df.shape    # Check the shape of the data
df.info()   # Check the info of the data
df.describe(include = 'all')   # Check the summary of the data
if df.isnull().any() == True:
    df.isnull().sum()   # Check the missing values
    print('There are missing values in the data')
ValueCounts_target = df['target'].value_counts()
ValueCounts_target    # Check the value counts of the target variable

###数据可视化###
# Find correlation
abs(df[df.columns[:]].corr()['target'][:].sort_values(ascending = False))      
corr = df.corr()
# plot the heatmap
plt.figure(figsize = (17,8))
sns.heatmap(corr, 
            xticklabels=corr.columns,
            linewidth=0.5,
            yticklabels=corr.columns,
            cmap="YlGnBu")
plt.show()

###数据建模###
y = df['target']
X_np = df.values
layers = []
current_layer = []
# Loop through all rows in X
for row in X_np:
    if row[-1] != 0x00:
        # If the target value is non-zero, add the current layer to the list of layers
        # and start a new current layer
        if current_layer:
            layers.append(np.array(current_layer))
        current_layer = [row]
    else:
        # If the target value is zero, add the row to the current layer
        current_layer.append(row)
# Add the last layer if it's not empty
if current_layer:
    layers.append(np.array(current_layer))
X = np.array(layers)    # Convert the list of 2D arrays into a 3D array
X = X.drop('target', axis=1, inplace=True)

###数据验证###
print(X[0].shape, X[1].shape)    # Verify.

###数据分割###
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X, y , test_size=0.20, random_state=42)
scaler = StandardScaler()  # Scale the data
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.fit_transform(X_test)

###模型训练###
Rf_classifier = RandomForestClassifier(n_estimators = 1000)
Rf_classifier.fit(X_train_scaled, y_train)

###模型评估###
y_pred_rf = Rf_classifier.predict(X_test_scaled)
accu_rf = accuracy_score(y_pred_rf, y_test)
print('Test Accuracy Random Forest Classifier :', accu_rf)
print('Confusion Matrix Random Forest Classifier :\n', confusion_matrix(y_pred_rf, y_test))