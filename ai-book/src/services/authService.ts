// ai-book/src/services/authService.ts
import { setAccessToken, removeAccessToken } from '../utils/authUtils';

const API_BASE_URL = 'http://localhost:8000/api/auth'; // FastAPI backend URL

interface AuthResponse {
  accessToken: string;
  // refreshToken?: string; // If using HttpOnly cookies, this won't be in JS
  user: {
    id: string;
    email: string;
    username?: string;
    roles?: string[];
  };
}

interface LoginRequest {
  email: string;
  password: string;
}

interface RegisterRequest extends LoginRequest {
  username?: string;
}

export const login = async (credentials: LoginRequest): Promise<AuthResponse> => {
  const response = await fetch(`${API_BASE_URL}/login`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(credentials),
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Login failed');
  }

  const data: AuthResponse = await response.json();
  setAccessToken(data.accessToken);
  return data;
};

export const register = async (userData: RegisterRequest): Promise<AuthResponse> => {
  const response = await fetch(`${API_BASE_URL}/register`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(userData),
  });

  if (!response.ok) {
    const errorData = await response.json();
    throw new Error(errorData.detail || 'Registration failed');
  }

  const data: AuthResponse = await response.json();
  setAccessToken(data.accessToken); // Log in user immediately after registration
  return data;
};

export const logout = async (): Promise<void> => {
  // Optionally call backend to invalidate token if needed
  // await fetch(`${API_BASE_URL}/logout`, { method: 'POST' });
  removeAccessToken();
};
