// ai-book/__tests__/hooks/useAuth.test.ts
import { renderHook, act, waitFor } from '@testing-library/react';
import { AuthProvider, useAuth } from '../../src/components/Auth/AuthProvider';
import * as authService from '../../src/services/authService';
import * as authUtils from '../../src/utils/authUtils';

jest.mock('../../src/services/authService');
jest.mock('../../src/utils/authUtils', () => ({
  getAccessToken: jest.fn(),
  setAccessToken: jest.fn(),
  removeAccessToken: jest.fn(),
}));

const mockAuthService = authService as jest.Mocked<typeof authService>;
const mockGetAccessToken = authUtils.getAccessToken as jest.Mock;
const mockSetAccessToken = authUtils.setAccessToken as jest.Mock;
const mockRemoveAccessToken = authUtils.removeAccessToken as jest.Mock;

describe('useAuth', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockGetAccessToken.mockReturnValue(null);
  });

  it('should return default unauthenticated state initially', () => {
    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.user).toBeNull();
    expect(result.current.isLoading).toBe(false); // Initial useEffect runs sync if no token
    expect(result.current.error).toBeNull();
  });

  it('should set isAuthenticated to true if token exists on mount', async () => {
    mockGetAccessToken.mockReturnValue('mock_token');
    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await waitFor(() => expect(result.current.isLoading).toBe(false));
    expect(result.current.isAuthenticated).toBe(true);
    expect(result.current.user).toEqual({ id: 'dummy_id', email: 'authenticated@example.com' });
  });

  it('should handle login successfully', async () => {
    const mockUser = { id: '1', email: 'test@example.com' };
    mockAuthService.login.mockResolvedValue({ accessToken: 'new_token', user: mockUser });

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await act(async () => {
      await result.current.login('test@example.com', 'password123');
    });

    expect(mockAuthService.login).toHaveBeenCalledWith({ email: 'test@example.com', password: 'password123' });
    expect(mockSetAccessToken).toHaveBeenCalledWith('new_token');
    expect(result.current.isAuthenticated).toBe(true);
    expect(result.current.user).toEqual(mockUser);
    expect(result.current.error).toBeNull();
  });

  it('should handle login failure', async () => {
    const errorMessage = 'Invalid credentials';
    mockAuthService.login.mockRejectedValue(new Error(errorMessage));

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await act(async () => {
      await expect(result.current.login('wrong@example.com', 'password')).rejects.toThrow(errorMessage);
    });

    expect(mockAuthService.login).toHaveBeenCalled();
    expect(mockSetAccessToken).not.toHaveBeenCalled();
    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.user).toBeNull();
    expect(result.current.error).toBe(errorMessage);
  });

  it('should handle registration successfully', async () => {
    const mockUser = { id: '2', email: 'new@example.com', username: 'newUser' };
    mockAuthService.register.mockResolvedValue({ accessToken: 'register_token', user: mockUser });

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await act(async () => {
      await result.current.register('new@example.com', 'newpassword', 'newUser');
    });

    expect(mockAuthService.register).toHaveBeenCalledWith({
      email: 'new@example.com',
      password: 'newpassword',
      username: 'newUser',
    });
    expect(mockSetAccessToken).toHaveBeenCalledWith('register_token');
    expect(result.current.isAuthenticated).toBe(true);
    expect(result.current.user).toEqual(mockUser);
    expect(result.current.error).toBeNull();
  });

  it('should handle registration failure', async () => {
    const errorMessage = 'Email already exists';
    mockAuthService.register.mockRejectedValue(new Error(errorMessage));

    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await act(async () => {
      await expect(result.current.register('existing@example.com', 'password')).rejects.toThrow(errorMessage);
    });

    expect(mockAuthService.register).toHaveBeenCalled();
    expect(mockSetAccessToken).not.toHaveBeenCalled();
    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.user).toBeNull();
    expect(result.current.error).toBe(errorMessage);
  });

  it('should handle logout successfully', async () => {
    mockAuthUtils.getAccessToken.mockReturnValue('existing_token');
    const { result } = renderHook(() => useAuth(), { wrapper: AuthProvider });

    await waitFor(() => expect(result.current.isAuthenticated).toBe(true));

    mockAuthService.logout.mockResolvedValue(undefined);

    await act(async () => {
      await result.current.logout();
    });

    expect(mockAuthService.logout).toHaveBeenCalled();
    expect(mockAuthUtils.removeAccessToken).toHaveBeenCalled();
    expect(result.current.isAuthenticated).toBe(false);
    expect(result.current.user).toBeNull();
    expect(result.current.error).toBeNull();
  });
});
