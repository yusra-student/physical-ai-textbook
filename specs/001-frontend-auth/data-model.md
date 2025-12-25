# Data Model: Frontend Authentication

**Feature**: 001-frontend-auth
**Date**: 2025-12-14

## Entities

### User

- **Purpose**: Represents an individual user account.
- **Attributes (Frontend Perspective)**:
    - `id`: Unique identifier for the user (string).
    - `email`: User's email address, used for login (string, unique).
    - `username`: Optional unique username (string).
    - `roles`: List of roles/permissions assigned to the user (array of strings, e.g., "admin", "viewer").
    - `isAuthenticated`: Boolean flag indicating if the user is currently logged in.
    - `accessToken`: JWT or similar token for API authorization (string, stored securely).
    - `refreshToken`: Token for acquiring new access tokens (string, managed by backend via HttpOnly cookies).

### Session

- **Purpose**: Represents an active authenticated user session.
- **Attributes (Frontend Perspective)**:
    - `accessToken`: Current access token (string).
    - `expiry`: Expiration time of the access token (timestamp).
    - `user`: Associated User entity.

## Relationships

- A `User` can have multiple `Session`s (e.g., across different devices/browsers).
- A `Session` belongs to one `User`.

## Validation Rules (Frontend)

- **Email**: Must be a valid email format.
- **Password**: Minimum length (e.g., 8 characters), must include uppercase, lowercase, number, special character (specific rules to be defined by backend API).
- **Password Confirmation**: Must match the password field during registration.
