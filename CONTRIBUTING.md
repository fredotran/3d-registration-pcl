# Contributing to 3D Registration PCL

Thank you for your interest in contributing to the 3D Registration PCL project! This document provides guidelines and instructions for contributing effectively.

## Code of Conduct

- Be respectful and inclusive in all interactions
- Provide constructive feedback and accept it gracefully
- Focus on what is best for the community and project
- Show empathy towards other contributors

## Development Workflow

### Setting Up the Development Environment

1. **Fork and Clone the Repository**
   ```bash
   git clone https://github.com/your-username/3d-registration-pcl.git
   cd 3d-registration-pcl
   ```

2. **Install Dependencies**
   ```bash
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git libeigen3-dev \
       libflann-dev libboost-all-dev libvtk7-dev libqhull-dev \
       libx11-dev libxi-dev libxrandr-dev libglew-dev libglu1-mesa-dev \
       libopenni-dev libopenni2-dev libpcl-dev pcl-tools libgtest-dev
   ```

3. **Build the Project**
   ```bash
   cmake -B build -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON
   cmake --build build
   ```

4. **Run Tests**
   ```bash
   cd build
   ctest --output-on-failure
   ```

### Branching Strategy

- `main` - Stable production code
- `develop` - Integration branch for features
- `feature/feature-name` - Feature branches
- `bugfix/bug-description` - Bug fix branches
- `hotfix/critical-fix` - Urgent production fixes

### Making Changes

1. **Create a Feature Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make Your Changes**
   - Write clean, readable code following the project's coding standards
   - Add tests for new functionality
   - Update documentation as needed
   - Run tests locally before committing

3. **Code Style**
   - Follow the existing code style (see `.clang-format`)
   - Use `clang-format` to format your code:
     ```bash
     find src include tests -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i
     ```
   - Run static analysis:
     ```bash
     cppcheck --enable=all --std=c++17 --inline-suppr -I include src/ include/
     ```

4. **Commit Your Changes**
   ```bash
   git add .
   git commit -m "feat: add your feature description"
   ```
   Use conventional commit messages:
   - `feat:` - New feature
   - `fix:` - Bug fix
   - `docs:` - Documentation changes
   - `test:` - Test changes
   - `refactor:` - Code refactoring
   - `style:` - Code style changes
   - `chore:` - Maintenance tasks

5. **Push and Create Pull Request**
   ```bash
   git push origin feature/your-feature-name
   ```
   Then create a pull request on GitHub with:
   - Clear description of changes
   - Related issue numbers
   - Testing performed
   - Screenshots if applicable

## Testing Guidelines

### Writing Tests

- Use Google Test framework
- Place test files in the `tests/` directory
- Name test files as `test_<module_name>.cpp`
- Write descriptive test names following the pattern `TEST(TestSuite, TestCase)`

### Test Coverage

- Aim for high test coverage on new code
- Test both success and failure cases
- Test edge cases and boundary conditions
- Use test fixtures for common setup

### Running Tests

```bash
# Run all tests
cd build
ctest

# Run specific test
./tests/pcl_registration_tests --gtest_filter=TestSuite.TestCase

# Run with verbose output
ctest --verbose
```

## Code Review Process

1. **Self-Review**
   - Ensure your code follows project guidelines
   - Run all tests locally
   - Check code formatting and style
   - Verify documentation is updated

2. **Peer Review**
   - Request review from maintainers
   - Address feedback constructively
   - Keep discussions focused and professional

3. **Approval**
   - Wait for approval from maintainers
   - Address any remaining concerns
   - Ensure CI checks pass

## Documentation

### Code Documentation

- Use Doxygen-style comments for public APIs
- Document function parameters and return values
- Add examples for complex functionality
- Update README.md for user-facing changes

### API Documentation

- Generate API documentation using Doxygen:
  ```bash
  doxygen Doxyfile
  ```
- Documentation will be generated in `docs/html/`

## Issue Reporting

### Bug Reports

When reporting bugs, include:
- Clear description of the problem
- Steps to reproduce
- Expected vs actual behavior
- Environment details (OS, PCL version, compiler)
- Relevant logs or error messages
- Minimal reproducible example if possible

### Feature Requests

When requesting features, include:
- Clear description of the desired feature
- Use case and motivation
- Proposed implementation approach (if known)
- Potential impact on existing functionality

## Release Process

Releases are managed by maintainers following semantic versioning:
- MAJOR version for incompatible API changes
- MINOR version for backwards-compatible functionality
- PATCH version for backwards-compatible bug fixes

## Getting Help

- Check existing documentation and issues
- Ask questions in GitHub Discussions
- Contact maintainers for urgent matters

## Additional Resources

- [PCL Documentation](https://pointclouds.org/documentation/)
- [Google Test Documentation](https://google.github.io/googletest/)
- [CMake Documentation](https://cmake.org/documentation/)
- [Doxygen Documentation](https://www.doxygen.nl/manual/)

## License

By contributing to this project, you agree that your contributions will be licensed under the project's existing license.
