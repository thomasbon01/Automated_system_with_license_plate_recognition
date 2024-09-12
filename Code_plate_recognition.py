import RPi.GPIO as GPIO
import cv2
import pytesseract
import time

# Configurați calea către executabilul Tesseract
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

# Calea către fișierul cu numerele de înmatriculare
fisier_numere = '/home/bontya/numere_inmatriculate.txt'

# Citește numerele de înmatriculare din fișier
def citeste_numere_inmatriculare(fisier):
    try:
        with open(fisier, 'r') as file:
            numere = file.read().splitlines()
        return set(numere)
    except FileNotFoundError:
        print(f"Eroare: Fișierul {fisier} nu a fost găsit.")
        return set()

numere_inmatriculare = citeste_numere_inmatriculare(fisier_numere)

# Capturare imagine de la camera USB folosind OpenCV
def capture_image_with_opencv(output_path):
    cap = cv2.VideoCapture(0)  # Prima cameră detectată
    if not cap.isOpened():
        print("Eroare la deschiderea camerei!")
        return None

    ret, frame = cap.read()
    if not ret:
        print("Eroare la capturarea imaginii.")
        return None

    cv2.imwrite(output_path, frame)
    cap.release()
    print(f"Imagine capturată și salvată ca '{output_path}'.")
    return output_path

# Preprocesarea imaginii și recunoașterea textului
def recunoaste_numar_inmatriculare(imagine_path):
    imagine = cv2.imread(imagine_path)
    gray = cv2.cvtColor(imagine, cv2.COLOR_BGR2GRAY)  # Conversie la gri
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # Reducerea zgomotului
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # Binarizarea imaginii
    dilated = cv2.dilate(binary, None, iterations=1)

    # Testăm diferite configurări ale Tesseract
    config = '--oem 3 --psm 6'  # OEM 3: Standard, PSM 6: Un bloc unic de text
    text = pytesseract.image_to_string(dilated, config=config)  # Recunoașterea textului

    # Opțional: salvarea imaginii procesate pentru debug
    cv2.imwrite("processed_image.jpg", dilated)

    return text.strip()

# Funcții GPIO
def setup_gpio():
    GPIO.setwarnings(False)
    in1 = 17
    in2 = 27
    en_a = 4
    red_light = 22  # GPIO pin for the red light
    green_light = 23  # GPIO pin for the green light
    trigger = 18  # GPIO pin for the trigger of HC-SR04
    echo = 24  # GPIO pin for the echo of HC-SR04

    GPIO.setmode(GPIO.BCM)  # Set the GPIO mode

    # Set up GPIO pins
    GPIO.setup(in1, GPIO.OUT)  # Setup motor control pins
    GPIO.setup(in2, GPIO.OUT)
    GPIO.setup(en_a, GPIO.OUT)
    GPIO.setup(red_light, GPIO.OUT)  # Setup red light pin
    GPIO.setup(green_light, GPIO.OUT)  # Setup green light pin
    GPIO.setup(trigger, GPIO.OUT)  # Setup trigger pin
    GPIO.setup(echo, GPIO.IN)  # Setup echo pin

    # Initializarea PWM a pinului en_a cu frecventa 100Hz
    power_a = GPIO.PWM(en_a, 100)
    power_a.start(0)  # Start with 0% duty cycle

    # Setarea inițială a output-ului
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(red_light, GPIO.LOW)
    GPIO.output(green_light, GPIO.LOW)

    return in1, in2, power_a, red_light, green_light, trigger, echo

def get_distance(trigger, echo):
    # Măsurarea distanței folosind senzorul HC-SR04.
    # Trimiterea unui puls de 10us prin pinul trigger
    GPIO.output(trigger, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(trigger, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trigger, GPIO.LOW)

    # Măsurarea lungimii pulsației din pinul echo
    pulse_start = time.time()
    while GPIO.input(echo) == GPIO.LOW:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(echo) == GPIO.HIGH:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Conversia în cm
    distance = round(distance, 2)
    return distance

def move_motor_one_direction(in1, in2, power_a):
    print("Moving motor in one direction")
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    power_a.ChangeDutyCycle(100)  # Rularea motorului la putere maximă
    time.sleep(5)  # Rotație pentru 5 secunde
    GPIO.output(in1, GPIO.LOW)  # Oprirea motorului după 5 secunde
    power_a.ChangeDutyCycle(0)  # Oprirea semnalului PWM

def move_motor_opposite_direction(in1, in2, power_a):
    print("Moving motor in opposite direction")
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    power_a.ChangeDutyCycle(100)
    time.sleep(5)
    GPIO.output(in2, GPIO.LOW)
    power_a.ChangeDutyCycle(0)

def main():
    try:
        # Încărcați numerele de înmatriculare din fișier
        plates = citeste_numere_inmatriculare(fisier_numere)

        # Configurați GPIO
        in1, in2, power_a, red_light, green_light, trigger, echo = setup_gpio()

        print("Aștept să se recunoască un număr de înmatriculare...")

        while True:
            try:
                # Capturare imagine cu OpenCV
                imagine_path = capture_image_with_opencv('captured_frame.jpg')
                if imagine_path is None:
                    time.sleep(1)
                    continue

                # Recunoaște textul în imagine folosind Tesseract
                text_recunoscut = recunoaste_numar_inmatriculare(imagine_path)
                print(f"Text recunoscut: {text_recunoscut}")

                # Verifică dacă textul recunoscut este în lista de numere de înmatriculare
                if any(plate in text_recunoscut for plate in plates):
                    print(f"Număr de înmatriculare recunoscut: {text_recunoscut}")

                    # Acționează motorul și luminile
                    GPIO.output(green_light, GPIO.HIGH)
                    GPIO.output(red_light, GPIO.LOW)
                    move_motor_one_direction(in1, in2, power_a)

                    # Măsoară distanța până la obiect
                    while True:
                        distance = get_distance(trigger, echo)
                        print(f"Distanța: {distance} cm")
                        if distance >= 30:
                            break
                        time.sleep(1)

                    move_motor_opposite_direction(in1, in2, power_a)
                    GPIO.output(green_light, GPIO.LOW)
                    GPIO.output(red_light, GPIO.HIGH)
                else:
                    print(f"Număr de înmatriculare necunoscut: {text_recunoscut}")
            except Exception as e:
                print(f"Eroare în bucla principală: {e}")
                break

    except KeyboardInterrupt:
        print("Program întrerupt de utilizator.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
