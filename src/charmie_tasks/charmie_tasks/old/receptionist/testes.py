"""import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_face = mp.solutions.face_detection

# Inicialize o módulo do MediaPipe para a detecção de pose do corpo
pose = mp_pose.Pose()

# Inicialize o módulo do MediaPipe para a detecção de face
face_detection = mp_face.FaceDetection()

# Inicialize a captura de vídeo
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        print("Não foi possível ler o quadro do vídeo.")
        break

    # Converta a imagem para RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Execute a detecção de pose do corpo
    results_pose = pose.process(image_rgb)

    # Verifique se a detecção de pose foi bem-sucedida
    if results_pose.pose_landmarks:
        # Obtenha as coordenadas dos keypoints do torso
        torso_landmarks = results_pose.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER:mp_pose.PoseLandmark.RIGHT_HIP]

        # Verifique se todos os keypoints do tronco estão sendo detectados
        if all(point.x != 0.0 and point.y != 0.0 and point.z != 0.0 for point in torso_landmarks):
            # Execute a detecção de face
            results_face = face_detection.process(image_rgb)

            # Verifique se a face não foi detectada
            if not results_face.detections:
                print("Suba!")  # Ação: Imprimir "Suba!"

    # Desenhe os keypoints do corpo na imagem
    mp_drawing.draw_landmarks(
        image, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS,
        mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
        mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2))

    # Mostrar a imagem com keypoints
    cv2.imshow('MediaPipe Body Keypoints', image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()"""

"""import cv2
import numpy as np
import tensorflow as tf

genderList = ['Male', 'Female']
model_loaded_gender = tf.keras.models.load_model('/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_gender.h5')
        



def detectGender(image):
        color_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img=cv2.resize(color_img,(224,224))
        normalized_img = img / 255.0
        expanded_img = np.expand_dims(normalized_img, axis=0)
        # Realizar a previsão usando o modelo carregado
        predictions = model_loaded_gender.predict(expanded_img)  # Substitua ... pelos dados da imagem pré-processada
        
        # Obter a previsão da raça
        gender_predominante_index = np.argmax(predictions, axis=1)
        gender_predominante = ['Female', 'Male'][gender_predominante_index[0]]
        
        # Verificar se a previsão está disponível
        if gender_predominante is not None:
            gender_dominante = gender_predominante
            error = 0
        else:
            gender_dominante = None
            error = 1
        
        return gender_dominante, error,predictions

image_path="/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/images/Jack.png"
image=cv2.imread(image_path)
gender, error ,predictions= detectGender(image)

print(gender)
print(predictions)"""


import cv2
import numpy as np
import tensorflow as tf

model_loaded_race = tf.keras.models.load_model('/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/modelo_raca.h5')   

def detectrace(image):
        
    color_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img=cv2.resize(color_img,(224,224))
    normalized_img = img / 255.0
    expanded_img = np.expand_dims(normalized_img, axis=0)
    # Realizar a previsão usando o modelo carregado
    predictions = model_loaded_race.predict(expanded_img)  # Substitua ... pelos dados da imagem pré-processada
        
    # Obter a previsão da race
    race_predominante_index = np.argmax(predictions, axis=1)
    race_predominante = ['Asian', 'Indian', 'Black or African Descendent', 'White', 'Middle Eastern', 'Latino Hispanic'][race_predominante_index[0]]
    
    # Verificar se a previsão está disponível
    if race_predominante is not None:
            if predictions[0] > 1.5:
                race_dominante = race_predominante[0]
                error=0
            else:
                race_dominante = race_predominante
                error = 0
    else:
        race_dominante = None
        error = 1
    
    return race_dominante, error, predictions

image_path="/home/charmie/charmie_ws/src/charmie_receptionist/charmie_receptionist/images/Olivia.png"
image=cv2.imread(image_path)
race, error ,predictions= detectrace(image)

print(race)
print(predictions)