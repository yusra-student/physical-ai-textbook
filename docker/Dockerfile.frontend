# Use an official Node runtime as a parent image
FROM node:18-alpine

# Set environment variables
ENV NODE_ENV=production

# Set work directory
WORKDIR /app

# Copy package.json and package-lock.json (if available)
COPY frontend/package*.json ./

# Install dependencies
RUN npm ci --only=production

# Copy the rest of the application
COPY frontend/ .

# Build the React app
RUN npm run build

# Install a simple server to serve the static files
RUN npm install -g serve

# Expose port
EXPOSE 3000

# Run the application
CMD ["serve", "-s", "build", "-l", "3000"]