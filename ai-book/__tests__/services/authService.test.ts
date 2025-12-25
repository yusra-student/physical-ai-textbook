// ai-book/__tests__/services/authService.test.ts
import { login, register, logout } from '../../src/services/authService';
import { setAccessToken, removeAccessToken, getAccessToken } from '../../src/utils/authUtils';

// Mock the fetch API
global.fetch = jest.fn();

describe('authService', () => {
  beforeEach(() => {
    (fetch as jest.Mock).mockClear();
    removeAccessToken(); // Clear any stored token before each test
  });

  describe('login', () => {
    it('should successfully log in a user and store the access token', async () => {
      const mockAccessToken = 'mock_access_token';
      const mockUser = { id: '1', email: 'test@example.com' };
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({ accessToken: mockAccessToken, user: mockUser }),
      });

      const result = await login({ email: 'test@example.com', password: 'password123' });

      expect(fetch).toHaveBeenCalledWith('/api/auth/login', expect.any(Object));
      expect(result.accessToken).toBe(mockAccessToken);
      expect(result.user).toEqual(mockUser);
      expect(getAccessToken()).toBe(mockAccessToken);
    });

    it('should throw an error if login fails', async () => {
      const mockError = { detail: 'Invalid credentials' };
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: false,
        json: async () => mockError,
      });

      await expect(login({ email: 'wrong@example.com', password: 'badpassword' })).rejects.toThrow(
        'Invalid credentials'
      );
      expect(getAccessToken()).toBeNull();
    });
  });

  describe('register', () => {
    it('should successfully register a user and store the access token', async () => {
      const mockAccessToken = 'mock_register_token';
      const mockUser = { id: '2', email: 'new@example.com', username: 'newUser' };
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({ accessToken: mockAccessToken, user: mockUser }),
      });

      const result = await register({ email: 'new@example.com', password: 'newpassword123', username: 'newUser' });

      expect(fetch).toHaveBeenCalledWith('/api/auth/register', expect.any(Object));
      expect(result.accessToken).toBe(mockAccessToken);
      expect(result.user).toEqual(mockUser);
      expect(getAccessToken()).toBe(mockAccessToken);
    });

    it('should throw an error if registration fails', async () => {
      const mockError = { detail: 'Email already in use' };
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: false,
        json: async () => mockError,
      });

      await expect(register({ email: 'existing@example.com', password: 'password' })).rejects.toThrow(
        'Email already in use'
      );
      expect(getAccessToken()).toBeNull();
    });
  });

  describe('logout', () => {
    it('should remove the access token', async () => {
      setAccessToken('some_token');
      expect(getAccessToken()).not.toBeNull();

      await logout();

      expect(getAccessToken()).toBeNull();
      // Expect fetch to not be called if backend invalidation is optional/not implemented yet
      expect(fetch).not.toHaveBeenCalled(); 
    });
  });
});
