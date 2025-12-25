/** @type {import('@jest/types').Config.InitialOptions} */
const config = {
    testEnvironment: 'jest-environment-jsdom',
    setupFilesAfterEnv: ['<rootDir>/jest.setup.js'],
    transform: {
      '^.+\.(ts|tsx)$': 'babel-jest',
      '^.+\.(js|jsx)$': 'babel-jest',
    },
    moduleNameMapper: {
      '^@/(.*)$': '<rootDir>/src/$1', // Map @/ to src/
      '^@docusaurus/theme-common$': '<rootDir>/node_modules/@docusaurus/theme-common',
    },
    testMatch: [
      '<rootDir>/__tests__/**/*.test.{js,jsx,ts,tsx}',
      '<rootDir>/src/**/*.{test,spec}.{js,jsx,ts,tsx}',
    ],
    moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json', 'node'],
    globals: {
      'ts-jest': {
        tsconfig: 'tsconfig.json',
      },
    },
  };
  
  module.exports = config;