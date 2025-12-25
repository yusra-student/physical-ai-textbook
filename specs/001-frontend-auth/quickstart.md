# Quickstart: Frontend Authentication

**Feature**: 001-frontend-auth
**Date**: 2025-12-14

## Setup Authentication Pages

To quickly get started with the new frontend authentication pages:

1.  **Ensure Backend API is Running**:
    Make sure your backend authentication API (as defined in `contracts/auth_api.yaml`) is accessible and running, listening on the expected `/api/auth` endpoint. For development, you might need to run the backend API separately.

2.  **Navigate to the AI Book Frontend**:
    The authentication pages will be integrated into the `ai-book` Docusaurus project.
    Navigate to the `ai-book` directory:
    ```bash
    cd ai-book
    ```

3.  **Install Dependencies**:
    If you haven't already, install the frontend dependencies:
    ```bash
    npm install
    ```

4.  **Start the Docusaurus Development Server**:
    This will launch the frontend application, including the new authentication pages.
    ```bash
    npm start
    ```

5.  **Access Authentication Pages**:
    Once the development server is running (usually at `http://localhost:3000`), you should be able to access the new authentication pages:
    -   **Login Page**: Navigate to `/login` (e.g., `http://localhost:3000/login`)
    -   **Registration Page**: Navigate to `/register` (e.g., `http://localhost:3000/register`)

## Testing the Authentication Flow

Follow these steps to test the core authentication flows:

1.  **User Registration**:
    -   Go to the registration page (`/register`).
    -   Enter a new email and password (ensure it meets any backend complexity requirements).
    -   Click the "Register" button.
    -   **Expected Result**: Successful registration should redirect you to the dashboard or login page, or log you in automatically, depending on the implementation.

2.  **User Login**:
    -   Go to the login page (`/login`).
    -   Enter the credentials of a registered user.
    -   Click the "Login" button.
    -   **Expected Result**: Successful login should redirect you to the dashboard or home page.

3.  **Access Protected Content**:
    -   After logging in, try to access a protected route (if any are implemented in the Docusaurus site).
    -   **Expected Result**: You should be able to view the protected content.

4.  **Logout**:
    -   Click the "Logout" button/link (if available in the UI).
    -   **Expected Result**: You should be redirected to the login page, and your session should be terminated.

## Troubleshooting

-   **Backend Not Running**: If you encounter network errors or "connection refused" messages, ensure your backend authentication API is running and accessible.
-   **Invalid Credentials**: Double-check the email and password used for login/registration. Refer to backend API documentation for password policy.
-   **Frontend Build Issues**: If the Docusaurus server fails to start, check the console for error messages and ensure all `npm install` dependencies are resolved.
