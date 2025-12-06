# Quickstart for Contributors

**Date**: 2025-12-05
**Context**: This guide provides instructions for setting up the development environment to contribute to the book.

## Prerequisites

1.  **Git**: [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
2.  **Node.js**: [Install Node.js](https://nodejs.org/en/download/) (v18 or higher)
3.  **Docker**: [Install Docker Desktop](https://www.docker.com/products/docker-desktop/)
4.  **VS Code**: [Install VS Code](https://code.visualstudio.com/download)
    - Recommended Extension: [MDX](https://marketplace.visualstudio.com/items?itemName=unifiedjs.vscode-mdx)

## 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

## 2. Install Docusaurus Dependencies

Navigate to the `ai-book` directory and install the necessary Node.js packages.

```bash
cd ai-book
npm install
```

## 3. Run the Development Server

Once the installation is complete, you can start the Docusaurus development server.

```bash
npm start
```

This will launch a local development server and open up a browser window. Most changes are reflected live without having to restart the server. The site will be available at `http://localhost:3000`.

## 4. Using the Docker Environment for Code Examples

To ensure all code examples work in a consistent environment, use the provided Docker setup.

### Build the Docker Image

From the root of the repository:

```bash
docker-compose build
```

### Run the Docker Container

This will give you a shell inside the container with all the necessary robotics (ROS 2, Gazebo) and AI (NVIDIA Isaac) libraries installed.

```bash
docker-compose run --rm book_env
```

You can now navigate to the `code-examples` directory within the container and run the examples for any given module.
