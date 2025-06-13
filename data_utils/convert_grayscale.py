import cv2
import numpy as np

def normalize_grayscale_to_max(image_path):
    """
    Normaliza a intensidade de uma imagem em escala de cinza de forma que o pixel mais forte
    (maior intensidade na imagem original) tenha intensidade 255, e o pixel mais fraco
    (menor intensidade na imagem original) tenha intensidade 0.

    Args:
        image_path (str): O caminho para a imagem em escala de cinza.

    Returns:
        numpy.ndarray: A imagem em escala de cinza normalizada.
    """
    try:
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if img is None:
            return "Erro: Não foi possível ler a imagem. Verifique o caminho do arquivo."

        # Encontra a intensidade mínima e máxima na imagem original
        min_val = np.min(img)
        max_val = np.max(img)

        # Se todos os pixels tiverem a mesma intensidade, não há o que normalizar, retorna a imagem como está
        if max_val == min_val:
            print("Atenção: Todos os pixels têm a mesma intensidade. Nenhuma normalização aplicada.")
            return img

        # Normaliza a imagem: (pixel - min_val) * (255 / (max_val - min_val))
        # Isso mapeia o range [min_val, max_val] para [0, 255]
        normalized_img = cv2.normalize(img, None, 0, 150, cv2.NORM_MINMAX)

        # Garante que o tipo de dado seja 8-bit unsigned integer (0-255)
        normalized_img = normalized_img.astype(np.uint8)

        return normalized_img

    except Exception as e:
        return f"Erro: Ocorreu um erro: {e}"

normalized_image = normalize_grayscale_to_max("/home/lh/Desktop/reuniao_10.06/rede_0.png") # Use dummy_image_path para teste ou seu 'image_path'

if isinstance(normalized_image, str):
    print(normalized_image)
else:
    output_filename = '/home/lh/Desktop/reuniao_10.06/normalized_image_max_to_2553.jpg'
    cv2.imwrite(output_filename, normalized_image)
    print(f"Imagem normalizada salva como '{output_filename}'")

    # Opcional: Para verificar as intensidades na imagem normalizada
    # print("\nValores de pixel da imagem normalizada (amostra):")
    # print(normalized_image)