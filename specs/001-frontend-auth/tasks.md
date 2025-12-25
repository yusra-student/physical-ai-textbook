# Tasks: Frontend Authentication

**Feature Branch**: `001-frontend-auth` | **Date**: 2025-12-14 | **Spec**: [specs/001-frontend-auth/spec.md](spec.md)
**Plan**: [specs/001-frontend-auth/plan.md](plan.md)
**Status**: Pending

## Phase 1: Setup

**Goal**: Prepare the Docusaurus project for frontend authentication development.

-   **Independent Test**: N/A

-   **Tasks**:
    - [x] T001 Create `Auth` directory for authentication components: `ai-book/src/components/Auth/`
    - [x] T002 Create `common` directory for shared components: `ai-book/src/components/common/`
    - [x] T003 Create `hooks` directory for React hooks: `ai-book/src/hooks/`
    - [x] T004 Create `pages` directory for authentication pages: `ai-book/src/pages/`
    - [x] T005 Create `services` directory for API services: `ai-book/src/services/`
    - [x] T006 Create `utils` directory for utility functions: `ai-book/src/utils/`
    - [x] T007 Configure Jest and React Testing Library for `ai-book` project in `ai-book/package.json`

## Phase 2: Foundational

**Goal**: Establish the core authentication logic and state management.

-   **Independent Test**: The `authService` can successfully make API calls to the backend and handle tokens. The `AuthProvider` can provide authentication state.

-   **Tasks**:
    - [x] T008 Implement `authUtils.ts` for token handling (storage, retrieval, validation) in `ai-book/src/utils/authUtils.ts`
    - [x] T009 Implement `authService.ts` for backend API calls (`/login`, `/register`, `/logout`) in `ai-book/src/services/authService.ts`
    - [x] T010 [P] Create `authService.test.ts` for `authService` unit tests in `ai-book/__tests__/services/authService.test.ts`
    - [x] T011 [P] Create `AuthProvider.tsx` (React Context/Provider) for global authentication state management in `ai-book/src/components/Auth/AuthProvider.tsx`
    - [x] T012 [P] Implement `useAuth.ts` hook for accessing authentication state and actions in `ai-book/src/hooks/useAuth.ts`
    - [x] T013 Create `useAuth.test.ts` for `useAuth` hook unit tests in `ai-book/__tests__/hooks/useAuth.test.ts`

## Phase 3: User Story 1 - User Login (Priority: P1)

**Goal**: Enable users to log into the application.

-   **Independent Test**: A user can successfully log in with valid credentials and be redirected to the dashboard.

-   **Tasks**:
    -   - [ ] T014 [US1] Create `LoginForm.tsx` component in `ai-book/src/components/Auth/LoginForm.tsx`
    -   - [ ] T015 [P] [US1] Create `login.tsx` page, integrating `LoginForm.tsx` and handling submission in `ai-book/src/pages/login.tsx`
    -   - [ ] T016 [P] [US1] Create `LoginForm.test.tsx` for `LoginForm` component unit tests in `ai-book/__tests__/components/Auth/LoginForm.test.ts`
    -   - [ ] T017 [US1] Implement login logic in `authService.ts` (if not already done in T009) to call `/login` endpoint.

## Phase 4: User Story 2 - User Registration (Priority: P1)

**Goal**: Allow new users to create accounts.

-   **Independent Test**: A new user can successfully register and either log in automatically or be redirected to the login page.

-   **Tasks**:
    -   - [ ] T018 [US2] Create `RegisterForm.tsx` component in `ai-book/src/components/Auth/RegisterForm.tsx`
    -   - [ ] T019 [P] [US2] Create `register.tsx` page, integrating `RegisterForm.tsx` and handling submission in `ai-book/src/pages/register.tsx`
    -   - [ ] T020 [P] [US2] Create `RegisterForm.test.tsx` for `RegisterForm` component unit tests in `ai-book/__tests__/components/Auth/RegisterForm.test.ts`
    -   - [ ] T021 [US2] Implement registration logic in `authService.ts` (if not already done in T009) to call `/register` endpoint.

## Phase 5: User Story 3 - Session Management (Priority: P2)

**Goal**: Securely manage user sessions, including logout and protected routes.

-   **Independent Test**: A logged-in user can log out, and authenticated routes are protected from unauthenticated access.

-   **Tasks**:
    -   - [ ] T022 [US3] Implement logout functionality in `authService.ts` to call `/logout` endpoint and clear local storage in `ai-book/src/services/authService.ts`
    -   - [ ] T023 [US3] Create `logout.tsx` page to handle session termination and redirection in `ai-book/src/pages/logout.tsx`
    -   - [ ] T024 [US3] Implement `ProtectedRoute.tsx` component to guard authenticated routes in `ai-book/src/components/common/ProtectedRoute.tsx`
    -   - [ ] T025 [P] [US3] Create `ProtectedRoute.test.tsx` for `ProtectedRoute` component unit tests in `ai-book/__tests__/components/common/ProtectedRoute.test.ts`
    -   - [ ] T026 [US3] Integrate `AuthProvider` at the root of the Docusaurus application (`ai-book/src/theme/Layout.tsx` or similar central component).

## Final Phase: Polish & Cross-Cutting Concerns

**Goal**: Ensure a complete, polished, and secure frontend authentication experience.

-   **Independent Test**: All authentication flows are robust, secure, and user-friendly.

-   **Tasks**:
    -   - [ ] T027 Integrate login/logout/register links into Docusaurus navigation in `ai-book/docusaurus.config.ts` or `ai-book/src/theme/NavbarItem.tsx`.
    -   - [ ] T028 Implement "Forgot Password" redirection from login page to an external or new internal page in `ai-book/src/pages/login.tsx`.
    -   - [ ] T029 Refine error message display and user feedback across all authentication forms.
    -   - [ ] T030 Conduct end-to-end integration tests for all authentication flows.
    -   - [ ] T031 Update `ai-book/README.md` with instructions for running and testing frontend authentication.

## Dependencies

-   Phase 1 (Setup) -> Phase 2 (Foundational)
-   Phase 2 (Foundational) -> Phase 3 (User Login)
-   Phase 2 (Foundational) -> Phase 4 (User Registration)
-   Phase 2 (Foundational) -> Phase 5 (Session Management)
-   Phase 3, 4, 5 -> Final Phase (Polish & Cross-Cutting Concerns)

## Parallel Execution Examples

-   **Within Phase 1**: Tasks T001-T006 (directory creation) can be done in parallel.
-   **Within Phase 2**: T010 (authService tests), T011 (AuthProvider), and T012 (useAuth hook) can be developed in parallel once T008 (authUtils) and T009 (authService initial structure) are done.
-   **Within Phase 3**: T015 (login page) and T016 (LoginForm tests) can be done in parallel with T014 (LoginForm component) as soon as the component is stubbed out.
-   **Within Phase 4**: T019 (register page) and T020 (RegisterForm tests) can be done in parallel with T018 (RegisterForm component) as soon as the component is stubbed out.

## Implementation Strategy

-   **MVP**: Complete Phase 1 and Phase 2, followed by User Story 1 (Phase 3). This provides core login functionality.
-   **Next Increment**: Add User Story 2 (Phase 4) for registration.
-   **Full Feature**: Complete User Story 3 (Phase 5) and the Final Phase for session management and polish.
