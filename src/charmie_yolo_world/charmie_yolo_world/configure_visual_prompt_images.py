import json
from pathlib import Path
import cv2

home = str(Path.home())
midpath_visual_prompt_images = "charmie_ws/src/charmie_yolo_world/charmie_yolo_world/visual_prompt_images"
IMG_DIR = home+'/'+midpath_visual_prompt_images+'/'

def choose_image_from_folder(folder: Path) -> Path:
    images = sorted([p for p in folder.iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp", ".webp")])
    if not images:
        raise FileNotFoundError(f"No images found in: {folder.resolve()}")

    print("\nImages in visual_prompt_images/:")
    for i, p in enumerate(images):
        print(f"  [{i}] {p.name}")

    while True:
        s = input("Choose image index: ").strip()
        if s.isdigit() and 0 <= int(s) < len(images):
            return images[int(s)]
        print("Invalid index. Try again.")

def main():
    img_path = choose_image_from_folder((Path(IMG_DIR)))

    img = cv2.imread(str(img_path))
    if img is None:
        raise FileNotFoundError(f"Could not read image: {img_path}")

    h, w = img.shape[:2]
    print(f"\nLoaded: {img_path.name}  ({w}x{h})")
    print("Draw bbox -> ENTER, then type label. Type 'done' to finish.\n")

    bboxes = []
    labels = []

    while True:
        roi = cv2.selectROI("Select bbox (ENTER=confirm, C=cancel)", img, fromCenter=False, showCrosshair=True)
        cv2.destroyAllWindows()

        x, y, bw, bh = roi
        if bw == 0 or bh == 0:
            label = input("Label (or 'done' to finish): ").strip()
            if label.lower() == "done":
                break
            print("No bbox saved. Try selecting again.\n")
            continue

        x1, y1, x2, y2 = float(x), float(y), float(x + bw), float(y + bh)

        label = input("Label for this bbox (or 'done' to finish): ").strip()
        if label.lower() == "done":
            break
        if not label:
            label = "object"

        bboxes.append([x1, y1, x2, y2])
        labels.append(label)
        print(f"Saved bbox: {bboxes[-1]}  label='{label}'\n")

        more = input("Add another bbox? [Y/n]: ").strip().lower()
        if more == "n":
            break

    if not bboxes:
        print("No bboxes saved. Exiting.")
        return

    # Make class ids sequential from labels
    label_to_cls = {}
    cls_ids = []
    for lab in labels:
        if lab not in label_to_cls:
            label_to_cls[lab] = len(label_to_cls)
        cls_ids.append(label_to_cls[lab])

    json_path = img_path.with_suffix(".json")

    payload = {
        "type": "yoloe_visual_prompt",
        "image": img_path.name,
        "image_dir": str(IMG_DIR),
        "width": int(w),
        "height": int(h),
        "classes": [{"cls": cid, "name": name} for name, cid in label_to_cls.items()],
        "bboxes": [{"cls": int(c), "xyxy": [float(v) for v in bb]} for bb, c in zip(bboxes, cls_ids)],
        "meta": {"notes": ""},
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"\n✅ Wrote JSON: {json_path}")
    print("Classes:", payload["classes"])
    print("BBoxes:", payload["bboxes"])

if __name__ == "__main__":
    main()
