import os
from Markdown2docx import Markdown2docx

def md_to_word(md_file_path, word_file_path):
    try:
        project = Markdown2docx(md_file_path)
        project.eat_soup()
        project.save_down(word_file_path)
        print(f"Success! Word file saved at: {word_file_path}")
    except Exception as e:
        print(f"Error converting to Word: {e}")

if __name__ == "__main__":
    md_file = r"D:\11\test_new_io_R_OTA\TEST\Protocol_Specification_V1.md"
    word_file = r"D:\11\test_new_io_R_OTA\XIEY\Protocol_Specification_V1_Updated.docx"
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(word_file), exist_ok=True)
    
    md_to_word(md_file, word_file)
