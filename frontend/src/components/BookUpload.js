import React, { useState } from 'react';
import { uploadBook, getIngestionStatus } from '../services/api';

const BookUpload = () => {
  const [file, setFile] = useState(null);
  const [bookId, setBookId] = useState('');
  const [title, setTitle] = useState('');
  const [author, setAuthor] = useState('');
  const [uploading, setUploading] = useState(false);
  const [message, setMessage] = useState('');
  const [jobId, setJobId] = useState(null);
  const [progress, setProgress] = useState(0);

  const handleFileChange = (e) => {
    setFile(e.target.files[0]);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!file || !bookId || !title) {
      setMessage('Please fill in all required fields');
      return;
    }

    setUploading(true);
    setMessage('');
    setProgress(0);

    try {
      const result = await uploadBook(file, bookId, title, author);
      setMessage(`Upload initiated! Job ID: ${result.job_id}`);
      setJobId(result.job_id);

      // Poll for status updates
      const interval = setInterval(async () => {
        try {
          const status = await getIngestionStatus(result.job_id);
          setProgress(status.progress || 0);

          if (status.status === 'completed') {
            setMessage(`Upload completed successfully! ${status.chunks_created} chunks created.`);
            clearInterval(interval);
          } else if (status.status === 'failed') {
            setMessage(`Upload failed: ${status.error_message}`);
            clearInterval(interval);
          }
        } catch (error) {
          console.error('Error getting status:', error);
          clearInterval(interval);
        }
      }, 3000); // Check status every 3 seconds
    } catch (error) {
      setMessage(`Upload error: ${error.message}`);
    } finally {
      setUploading(false);
    }
  };

  return (
    <div>
      <h2>Upload Book for AI Processing</h2>
      <form id="upload-form" onSubmit={handleSubmit}>
        <div>
          <label>Book ID: *</label>
          <input
            type="text"
            value={bookId}
            onChange={(e) => setBookId(e.target.value)}
            required
          />
        </div>
        <div>
          <label>Title: *</label>
          <input
            type="text"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            required
          />
        </div>
        <div>
          <label>Author:</label>
          <input
            type="text"
            value={author}
            onChange={(e) => setAuthor(e.target.value)}
          />
        </div>
        <div>
          <label>File (PDF/EPUB): *</label>
          <input
            type="file"
            accept=".pdf,.epub"
            onChange={handleFileChange}
            required
          />
        </div>
        <button type="submit" disabled={uploading}>
          {uploading ? 'Uploading...' : 'Upload Book'}
        </button>
      </form>
      {message && <div className="results">{message}</div>}
      {progress > 0 && progress < 100 && (
        <div className="results">
          <p>Progress: {progress}%</p>
          <div style={{ width: '100%', backgroundColor: '#ddd' }}>
            <div
              style={{
                width: `${progress}%`,
                height: '20px',
                backgroundColor: '#4CAF50',
                textAlign: 'center',
                lineHeight: '20px',
                color: 'white'
              }}
            >
              {progress}%
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default BookUpload;