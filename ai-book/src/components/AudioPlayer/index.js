import React from 'react';

export default function AudioPlayer({ src }) {
  return (
    <audio controls src={src}>
      Your browser does not support the
      <code>audio</code> element.
    </audio>
  );
}
