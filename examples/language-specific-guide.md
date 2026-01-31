# Language-Specific Guide

프로그래밍 언어별 Claude Code 활용 가이드입니다.

## TypeScript / JavaScript

### 프로젝트 설정
```bash
# 권장 도구
npm install -D typescript eslint prettier @types/node

# tsconfig.json 생성
npx tsc --init
```

### 권장 에이전트
| 작업 | 에이전트 |
|------|----------|
| 아키텍처 설계 | `architect` |
| 코드 리뷰 | `code-reviewer` |
| 테스트 작성 | `test-writer` |
| 타입 개선 | `/type-check-improve` |

### 프롬프트 예시
```
Review this TypeScript code for:
- Type safety issues (any, unknown usage)
- Null/undefined handling
- Generic type usage
- Module organization
```

### 테스트 프레임워크
- **Jest**: 단위 테스트, 스냅샷 테스트
- **Vitest**: Vite 프로젝트
- **Playwright**: E2E 테스트

---

## Python

### 프로젝트 설정
```bash
# 권장 도구
pip install ruff mypy pytest black

# pyproject.toml 생성
```

### 권장 에이전트
| 작업 | 에이전트 |
|------|----------|
| 코드 리뷰 | `code-reviewer` |
| 테스트 작성 | `test-writer` |
| 리팩토링 | `refactorer` |
| 타입 힌트 | `/type-check-improve` |

### 프롬프트 예시
```
Review this Python code for:
- PEP 8 compliance
- Type hints completeness
- Exception handling
- Documentation strings
```

### 테스트 프레임워크
- **pytest**: 표준 테스트
- **unittest**: 내장 테스트
- **hypothesis**: 속성 기반 테스트

---

## Go

### 프로젝트 설정
```bash
# 프로젝트 초기화
go mod init myproject

# 권장 도구
go install golang.org/x/tools/cmd/goimports@latest
go install github.com/golangci/golangci-lint/cmd/golangci-lint@latest
```

### 권장 에이전트
| 작업 | 에이전트 |
|------|----------|
| 아키텍처 | `architect` |
| 코드 리뷰 | `code-reviewer` |
| 테스트 | `test-writer` |

### 프롬프트 예시
```
Review this Go code for:
- Error handling patterns
- Goroutine safety
- Interface design
- Package organization
```

### 테스트
```bash
go test ./...
go test -race ./...
go test -cover ./...
```

---

## Rust

### 프로젝트 설정
```bash
# 프로젝트 생성
cargo new myproject

# 권장 도구
rustup component add clippy rustfmt
```

### 권장 에이전트
| 작업 | 에이전트 |
|------|----------|
| 설계 | `architect` |
| 리뷰 | `code-reviewer` |
| 테스트 | `test-writer` |

### 프롬프트 예시
```
Review this Rust code for:
- Ownership and borrowing issues
- Error handling with Result/Option
- Lifetime annotations
- Unsafe code usage
```

### 테스트
```bash
cargo test
cargo clippy
cargo fmt --check
```

---

## 언어별 공통 패턴

### 1. 코드 리뷰 요청
```
Use the code-reviewer agent to review [file] focusing on:
- [Language]-specific best practices
- Performance considerations
- Error handling
- Test coverage
```

### 2. 테스트 작성 요청
```
Use the test-writer agent to add tests for [file]:
- Unit tests for each public function
- Edge cases and error conditions
- [Language]-specific testing patterns
```

### 3. 리팩토링 요청
```
Use the refactorer agent to improve [file]:
- Apply [Language] idioms
- Improve type safety
- Reduce code duplication
- Enhance readability
```

---

## 언어별 린터 설정

### TypeScript
```json
// .eslintrc.json
{
  "extends": ["eslint:recommended", "plugin:@typescript-eslint/recommended"],
  "parser": "@typescript-eslint/parser"
}
```

### Python
```toml
# pyproject.toml
[tool.ruff]
line-length = 88
select = ["E", "F", "W", "I", "N"]
```

### Go
```yaml
# .golangci.yml
linters:
  enable:
    - gofmt
    - govet
    - errcheck
    - staticcheck
```

### Rust
```toml
# .clippy.toml
cognitive-complexity-threshold = 25
```
