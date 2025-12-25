import PyPDF2
from ebooklib import epub
import ebooklib
from bs4 import BeautifulSoup
import tiktoken
from typing import List, Dict
import re


class TextProcessingService:
    def __init__(self, chunk_size: int = 512, overlap: int = 256):
        self.chunk_size = chunk_size
        self.overlap = overlap
        self.tokenizer = tiktoken.encoding_for_model("gpt2")  # Using gpt2 tokenizer as proxy

    async def extract_text_from_pdf(self, file) -> str:
        """Extract text from PDF file"""
        text = ""
        pdf_reader = PyPDF2.PdfReader(file)
        for page in pdf_reader.pages:
            text += page.extract_text() + "\n"
        return text

    async def extract_text_from_epub(self, file_path: str) -> str:
        """Extract text from EPUB file"""
        text = ""
        book = epub.read_epub(file_path)

        for item in book.get_items():
            if item.get_type() == ebooklib.ITEM_DOCUMENT:
                soup = BeautifulSoup(item.get_content(), 'html.parser')
                text += soup.get_text() + "\n"

        return text

    async def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[Dict]:
        """Split text into chunks with specified size and overlap"""
        if chunk_size is None:
            chunk_size = self.chunk_size
        if overlap is None:
            overlap = self.overlap

        # Split the text into paragraphs first
        paragraphs = text.split('\n\n')

        chunks = []
        current_chunk = ""
        chunk_index = 0

        for paragraph in paragraphs:
            # If this paragraph alone is larger than chunk_size, we need to split it further
            if self._count_tokens(paragraph) > chunk_size:
                # Split the large paragraph into smaller parts
                sub_chunks = self._split_large_text(paragraph, chunk_size, overlap)
                for sub_chunk in sub_chunks:
                    chunks.append({
                        "index": chunk_index,
                        "text": sub_chunk,
                        "tokens": self._count_tokens(sub_chunk)
                    })
                    chunk_index += 1
            else:
                # Check if adding this paragraph exceeds chunk size
                if self._count_tokens(current_chunk + paragraph) > chunk_size:
                    # Save the current chunk if it's substantial
                    if current_chunk.strip():
                        chunks.append({
                            "index": chunk_index,
                            "text": current_chunk.strip(),
                            "tokens": self._count_tokens(current_chunk)
                        })
                        chunk_index += 1

                    # Start a new chunk with overlap from the previous chunk
                    if current_chunk:
                        # Include overlap from the end of current chunk
                        overlap_text = self._get_overlap_text(current_chunk, overlap)
                        current_chunk = overlap_text + paragraph
                    else:
                        current_chunk = paragraph
                else:
                    current_chunk += "\n\n" + paragraph

        # Don't forget the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "index": chunk_index,
                "text": current_chunk.strip(),
                "tokens": self._count_tokens(current_chunk)
            })

        return chunks

    def _count_tokens(self, text: str) -> int:
        """Count tokens using tiktoken"""
        if not text:
            return 0
        return len(self.tokenizer.encode(text))

    def _get_overlap_text(self, text: str, overlap_tokens: int) -> str:
        """Extract the last overlap_tokens tokens from text"""
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= overlap_tokens:
            return text

        # Decode the last overlap_tokens tokens
        overlap_tokens_list = tokens[-overlap_tokens:]
        return self.tokenizer.decode(overlap_tokens_list)

    def _split_large_text(self, text: str, chunk_size: int, overlap: int) -> List[str]:
        """Split a large text block into smaller chunks"""
        sentences = re.split(r'[.!?]+', text)
        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            if self._count_tokens(current_chunk + sentence) <= chunk_size:
                current_chunk += sentence + ". "
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())

                # If sentence is longer than chunk_size, split it by words
                if self._count_tokens(sentence) > chunk_size:
                    words = sentence.split()
                    temp_chunk = ""
                    for word in words:
                        if self._count_tokens(temp_chunk + word) <= chunk_size:
                            temp_chunk += word + " "
                        else:
                            if temp_chunk:
                                chunks.append(temp_chunk.strip())
                                temp_chunk = word + " "
                            else:
                                # If even a single word is too large, force split it
                                chunks.extend(self._force_split_sentence(word, chunk_size))
                    current_chunk = temp_chunk
                else:
                    current_chunk = sentence + ". "

        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    def _force_split_sentence(self, sentence: str, chunk_size: int) -> List[str]:
        """Force split a sentence that is too long"""
        tokens = self.tokenizer.encode(sentence)
        chunks = []

        for i in range(0, len(tokens), chunk_size):
            chunk_tokens = tokens[i:i+chunk_size]
            chunk_text = self.tokenizer.decode(chunk_tokens)
            if chunk_text.strip():
                chunks.append(chunk_text.strip())

        return chunks


# Initialize globally
text_processing_service = TextProcessingService()