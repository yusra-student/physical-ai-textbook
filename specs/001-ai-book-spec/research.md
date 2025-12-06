# Research & Decisions

**Date**: 2025-12-05
**Context**: Initial planning for the "Physical AI and Humanoid Robotics" book.

This document records key technical decisions made during the planning phase.

---

### Decision: Docusaurus Plugins

**Decision**: The following Docusaurus plugins will be installed and configured:
- `@docusaurus/plugin-pwa`: To provide Progressive Web App capabilities, allowing for offline access to the book content.
- `docusaurus-plugin-image-zoom`: To allow readers to zoom in on images and diagrams for better clarity.
- `docusaurus-plugin-search-local` or `Algolia`: We will start with the local search plugin for simplicity and cost-effectiveness. If performance becomes an issue as content grows, we will migrate to Algolia.

**Rationale**: These plugins provide a significant improvement to the reader's experience with minimal configuration overhead. Local search is preferred initially to avoid reliance on external services and associated costs.

**Alternatives considered**:
- **Building custom components**: Building custom components for these features would be time-consuming and is not a core focus of the project.
- **Using other plugins**: The selected plugins are popular, well-maintained, and known to work well with the current version of Docusaurus.

---

### Decision: CI/CD and Hosting Platform

**Decision**: The project will be hosted on **Vercel**, and the CI/CD pipeline will be managed using **GitHub Actions**.

**Rationale**:
- **Vercel**: Offers a seamless deployment experience for Docusaurus projects, with automatic builds, previews for pull requests, and a generous free tier. It simplifies the process of getting a custom domain set up.
- **GitHub Actions**: Tightly integrated with the source code repository. It will be used to run automated checks on every pull request, including code linting, markdown link checking, and a Docusaurus build test to ensure the site remains functional.

**Alternatives considered**:
- **Netlify**: A strong alternative to Vercel with a similar feature set. Vercel is chosen for its slightly more streamlined UI and focus on Next.js/React-based frameworks like Docusaurus.
- **Jenkins/CircleCI**: More powerful but also more complex CI/CD tools. GitHub Actions is sufficient for the needs of this project and easier to configure.
