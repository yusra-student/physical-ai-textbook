// ai-book/src/utils/authUtils.ts

const ACCESS_TOKEN_KEY = 'accessToken';

export const getAccessToken = (): string | null => {
  return localStorage.getItem(ACCESS_TOKEN_KEY);
};

export const setAccessToken = (token: string): void => {
  localStorage.setItem(ACCESS_TOKEN_KEY, token);
};

export const removeAccessToken = (): void => {
  localStorage.removeItem(ACCESS_TOKEN_KEY);
};

export const isAuthenticated = (): boolean => {
  const token = getAccessToken();
  // In a real application, you would also validate the token's expiry
  // and potentially make a call to the backend to verify its validity.
  return !!token;
};
