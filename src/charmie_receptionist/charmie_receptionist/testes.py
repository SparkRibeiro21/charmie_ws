import cv2
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
cv2.destroyAllWindows()