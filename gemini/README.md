# Gemini Code Configuration Examples

This directory contains example workflows and configurations for Gemini Code.

## Workflows (`/workflows`)

Custom workflows allow you to automate common tasks using the Gemini Agent. 
To use these, copy the `.md` files into your project's `.agent/workflows/` directory.

- **`boot.md`**: Bootstraps a new project with standard structure and git init.
- **`review.md`**: Performs a code review on modified files.
- **`docs.md`**: Updates project documentation based on the codebase.

## Usage

1. Create a `.agent/workflows` directory in your project root.
2. Copy the desired workflow files into that directory.
3. In Gemini Code, use the slash command (e.g., `/boot`, `/review`) to trigger the workflow.
