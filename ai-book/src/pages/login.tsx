import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../hooks/useAuth';
import { useHistory } from '@docusaurus/router';
import Link from '@docusaurus/Link';

export default function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const { login, error, isLoading, isAuthenticated } = useAuth();
  const history = useHistory();

  // Redirect if already authenticated
  if (isAuthenticated) {
    history.push('/');
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      await login(email, password);
      history.push('/');
    } catch (err) {
      console.error(err);
    }
  };

  return (
    <Layout title="Login" description="Login to AI Book">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Login</h2>
              </div>
              <div className="card__body">
                <form onSubmit={handleSubmit}>
                  <div className="margin-bottom--md">
                    <label htmlFor="email" style={{display: 'block', marginBottom: '5px'}}>Email</label>
                    <input
                      type="email"
                      id="email"
                      style={{width: '100%', padding: '10px', borderRadius: '4px', border: '1px solid var(--ifm-color-emphasis-300)'}}
                      value={email}
                      onChange={(e) => setEmail(e.target.value)}
                      required
                    />
                  </div>
                  <div className="margin-bottom--md">
                    <label htmlFor="password" style={{display: 'block', marginBottom: '5px'}}>Password</label>
                    <input
                      type="password"
                      id="password"
                      style={{width: '100%', padding: '10px', borderRadius: '4px', border: '1px solid var(--ifm-color-emphasis-300)'}}
                      value={password}
                      onChange={(e) => setPassword(e.target.value)}
                      required
                    />
                  </div>
                  {error && <div className="alert alert--danger margin-bottom--md">{error}</div>}
                  <button type="submit" className="button button--primary button--block" disabled={isLoading}>
                    {isLoading ? 'Logging in...' : 'Login'}
                  </button>
                </form>
              </div>
              <div className="card__footer text--center">
                <p>Don't have an account? <Link to="/register">Register</Link></p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}
