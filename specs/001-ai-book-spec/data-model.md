# Data Model: Book Structure

**Date**: 2025-12-05
**Context**: This document defines the structure and entities for the book content.

## Core Entity: Lesson

Each `.mdx` file within the `docs/` directory represents a single `Lesson`.

### Attributes

- **`title`** (string, required): The title of the lesson, defined in the frontmatter.
- **`description`** (string, optional): A brief description of the lesson, defined in the frontmatter.
- **`content`** (MDX, required): The body of the lesson, following the standard format.
- **`order`** (integer, required): The order of the lesson within its module, determined by the `sidebar.js` configuration.

### Structure

Each lesson will adhere to the following internal structure:
1.  **Introduction**: A brief overview of what the lesson will cover and the learning objectives.
2.  **Theory**: The core theoretical concepts.
3.  **Practical Examples**: Code blocks and hands-on exercises to apply the theory.
4.  **Exercises**: A set of problems or questions for the reader to solve.

## Core Entity: Module

A `Module` is a logical grouping of `Lesson`s, represented by a subdirectory within the `docs/` directory.

### Attributes

- **`title`** (string, required): The title of the module, defined in the `sidebar.js` configuration.
- **`lessons`** (Array<Lesson>, required): An ordered list of lessons that belong to the module.

## Core Entity: Quiz

A `Quiz` is an interactive component associated with each `Module`.

### Attributes

- **`questions`** (Array<Question>, required): A list of 10 questions.
- **`Question`**:
    - `text` (string, required)
    - `options` (Array<string>, required)
    - `correct_answer` (integer, required): The index of the correct option.

## Relationships

- A `Book` has 4 `Module`s.
- Each `Module` has 3 `Lesson`s.
- Each `Module` has one `Quiz`.
