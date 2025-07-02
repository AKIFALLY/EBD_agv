import fitz  # PyMuPDF
import pytesseract
from PIL import Image
import io
from pathlib import Path

# PDF 路徑（請改成你的檔案路徑）
pdf_path = "KUKA_API.pdf"
output_md = "api_ocr.md"

# 設定語言：中文簡體 + 英文
ocr_lang = "chi_sim+eng"

# 儲存所有頁面 OCR 結果
ocr_text = ""

# 開啟 PDF 並逐頁處理
doc = fitz.open(pdf_path)
for i in range(len(doc)):
    page = doc.load_page(i)
    pix = page.get_pixmap(dpi=300)
    img = Image.open(io.BytesIO(pix.tobytes("png")))
    text = pytesseract.image_to_string(img, lang=ocr_lang)
    ocr_text += f"\n\n## Page {i+1}\n\n{text.strip()}"

# 輸出成 Markdown
Path(output_md).write_text(ocr_text, encoding="utf-8")
print(f"OCR 完成！已輸出到：{output_md}")
