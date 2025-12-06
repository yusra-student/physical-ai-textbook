import React from 'react';

export default function ThreeDViewer({ src, alt }) {
  return (
    <div style={{ height: '400px', width: '100%' }}>
      <model-viewer
        src={src}
        alt={alt}
        auto-rotate
        camera-controls
        style={{ height: '100%', width: '100%' }}
      ></model-viewer>
    </div>
  );
}
