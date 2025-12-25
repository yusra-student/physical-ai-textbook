// ai-book/src/components/Auth/AuthProvider.tsx
import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import { getAccessToken, setAccessToken, removeAccessToken } from '../../utils/authUtils';
import { login as apiLogin, register as apiRegister, logout as apiLogout } from '../../services/authService';

interface User {
  id: string;
  email: string;
  username?: string;
  roles?: string[];
}

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, password: string, username?: string) => Promise<void>;
  logout: () => Promise<void>;
  isLoading: boolean;
  error: string | null;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const checkAuth = async () => {
      const token = getAccessToken();
      if (token) {
        // In a real app, you'd verify the token with the backend
        // For now, we assume a token means authenticated
        // and fetch user info or decode token if needed
        // For this prototype, we'll just set a dummy user
        setUser({ id: 'dummy_id', email: 'authenticated@example.com' }); // Placeholder user
        setIsAuthenticated(true);
      }
      setIsLoading(false);
    };
    checkAuth();
  }, []);

  const login = async (email: string, password: string) => {
    setIsLoading(true);
    setError(null);
    try {
      const authResponse = await apiLogin({ email, password });
      setUser(authResponse.user);
      setIsAuthenticated(true);
    } catch (err: any) {
      setError(err.message || 'Login failed');
      throw err; // Re-throw to allow component to handle
    } finally {
      setIsLoading(false);
    }
  };

  const register = async (email: string, password: string, username?: string) => {
    setIsLoading(true);
    setError(null);
    try {
      const authResponse = await apiRegister({ email, password, username });
      setUser(authResponse.user);
      setIsAuthenticated(true);
    } catch (err: any) {
      setError(err.message || 'Registration failed');
      throw err; // Re-throw to allow component to handle
    } finally {
      setIsLoading(false);
    }
  };

  const logout = async () => {
    setIsLoading(true);
    setError(null);
    try {
      await apiLogout();
      setUser(null);
      setIsAuthenticated(false);
    } catch (err: any) {
      setError(err.message || 'Logout failed');
    } finally {
      setIsLoading(false);
    }
  };

  const authContextValue: AuthContextType = {
    user,
    isAuthenticated,
    login,
    register,
    logout,
    isLoading,
    error,
  };

  return <AuthContext.Provider value={authContextValue}>{children}</AuthContext.Provider>;
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};
