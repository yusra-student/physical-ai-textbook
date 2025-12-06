import React from 'react';

export default function CodeSandbox({children}) {
  return (
    <div style={{border: '1px solid #ccc', padding: '1rem', borderRadius: '4px'}}>
      {children}
    </div>
  );
}
