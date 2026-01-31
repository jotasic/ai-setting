# Security Hardening Guide

보안 강화를 위한 체크리스트와 가이드입니다.

## OWASP Top 10 체크리스트

### 1. Injection (A03:2021)

#### 위험
- SQL Injection
- NoSQL Injection
- Command Injection
- LDAP Injection

#### 검사 프롬프트
```
Use the security-auditor agent to check for injection vulnerabilities:
- SQL queries with string concatenation
- Shell command execution with user input
- Dynamic query building
```

#### 수정 방법
```typescript
// Bad
const query = `SELECT * FROM users WHERE id = ${userId}`;

// Good
const query = 'SELECT * FROM users WHERE id = $1';
const result = await db.query(query, [userId]);
```

---

### 2. Broken Authentication (A07:2021)

#### 체크리스트
- [ ] 강력한 비밀번호 정책
- [ ] 계정 잠금 (brute force 방지)
- [ ] 안전한 세션 관리
- [ ] MFA 지원
- [ ] 비밀번호 해싱 (bcrypt, argon2)

#### 검사 프롬프트
```
Use the security-auditor agent to review authentication:
- Password hashing algorithm
- Session token generation
- Login rate limiting
- Password reset flow
```

#### 수정 방법
```typescript
// Password hashing
import bcrypt from 'bcrypt';
const hash = await bcrypt.hash(password, 12);

// Session security
app.use(session({
  secret: process.env.SESSION_SECRET,
  cookie: {
    httpOnly: true,
    secure: true,
    sameSite: 'strict',
    maxAge: 3600000
  }
}));
```

---

### 3. Cross-Site Scripting (XSS) (A03:2021)

#### 유형
- Reflected XSS
- Stored XSS
- DOM-based XSS

#### 검사 프롬프트
```
Use the security-auditor agent to check for XSS:
- User input rendered without escaping
- dangerouslySetInnerHTML usage
- eval() or innerHTML usage
- URL parameter reflection
```

#### 수정 방법
```typescript
// React - automatic escaping
<div>{userInput}</div>

// If HTML needed, sanitize first
import DOMPurify from 'dompurify';
<div dangerouslySetInnerHTML={{
  __html: DOMPurify.sanitize(userHtml)
}} />
```

---

### 4. Insecure Direct Object References (A01:2021)

#### 검사 프롬프트
```
Use the security-auditor agent to check authorization:
- API endpoints accessing resources by ID
- Missing ownership verification
- Predictable resource IDs
```

#### 수정 방법
```typescript
// Bad
app.get('/api/orders/:id', async (req, res) => {
  const order = await Order.findById(req.params.id);
  res.json(order);
});

// Good
app.get('/api/orders/:id', async (req, res) => {
  const order = await Order.findOne({
    _id: req.params.id,
    userId: req.user.id  // Verify ownership
  });
  if (!order) return res.status(404).json({ error: 'Not found' });
  res.json(order);
});
```

---

### 5. Security Misconfiguration (A05:2021)

#### 체크리스트
- [ ] 프로덕션에서 디버그 모드 비활성화
- [ ] 불필요한 기능/포트 비활성화
- [ ] 기본 자격 증명 변경
- [ ] 보안 헤더 설정
- [ ] 에러 메시지에서 민감 정보 제거

#### 보안 헤더 설정
```typescript
import helmet from 'helmet';

app.use(helmet({
  contentSecurityPolicy: {
    directives: {
      defaultSrc: ["'self'"],
      scriptSrc: ["'self'"],
      styleSrc: ["'self'", "'unsafe-inline'"],
    }
  },
  hsts: { maxAge: 31536000, includeSubDomains: true }
}));
```

---

### 6. Sensitive Data Exposure (A02:2021)

#### 체크리스트
- [ ] HTTPS 강제
- [ ] 민감 데이터 암호화
- [ ] 로그에서 민감 정보 제거
- [ ] API 응답에서 불필요한 필드 제거

#### 검사 프롬프트
```
Use the security-auditor agent to check data exposure:
- API responses with sensitive fields
- Logging of passwords/tokens
- Unencrypted data storage
- .env files in repository
```

---

## 비밀 관리

### 환경 변수
```bash
# .env.example (commit this)
DATABASE_URL=
API_KEY=
JWT_SECRET=

# .env (never commit)
DATABASE_URL=postgres://...
API_KEY=sk-...
JWT_SECRET=...
```

### Git Hooks로 비밀 유출 방지
```bash
# .git/hooks/pre-commit
#!/bin/bash
if git diff --cached --name-only | xargs grep -l 'API_KEY\|SECRET\|PASSWORD' 2>/dev/null; then
  echo "Error: Possible secret in commit"
  exit 1
fi
```

### 비밀 스캔
```bash
# Git secrets
git secrets --install
git secrets --scan

# Gitleaks
gitleaks detect --source .
```

---

## 의존성 보안

### 취약점 스캔
```bash
# npm
npm audit
npm audit fix

# Python
pip-audit
safety check

# Snyk
snyk test
```

### 자동화
```yaml
# .github/workflows/security.yml
name: Security Scan
on: [push, pull_request]
jobs:
  audit:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: npm audit --audit-level=high
      - uses: snyk/actions/node@master
        env:
          SNYK_TOKEN: ${{ secrets.SNYK_TOKEN }}
```

---

## 보안 감사 워크플로우

```
1. security-auditor로 코드 스캔
   ↓
2. 취약점 목록 생성
   ↓
3. 심각도별 우선순위 지정
   - Critical: 즉시 수정
   - High: 1주일 내
   - Medium: 다음 릴리즈
   - Low: 백로그
   ↓
4. 수정 구현
   ↓
5. 재스캔으로 검증
```

---

## 보안 헤더 체크리스트

| 헤더 | 값 | 목적 |
|------|-----|------|
| Strict-Transport-Security | max-age=31536000 | HTTPS 강제 |
| X-Content-Type-Options | nosniff | MIME 스니핑 방지 |
| X-Frame-Options | DENY | 클릭재킹 방지 |
| Content-Security-Policy | default-src 'self' | XSS 방지 |
| X-XSS-Protection | 1; mode=block | XSS 필터 |
| Referrer-Policy | strict-origin | 리퍼러 노출 제한 |

---

## 인증/인가 패턴

### JWT Best Practices
```typescript
// Token 생성
const token = jwt.sign(
  { userId: user.id, role: user.role },
  process.env.JWT_SECRET,
  { expiresIn: '1h', algorithm: 'RS256' }
);

// Token 검증
const decoded = jwt.verify(token, process.env.JWT_PUBLIC_KEY, {
  algorithms: ['RS256']
});
```

### RBAC 구현
```typescript
const permissions = {
  admin: ['read', 'write', 'delete', 'manage'],
  editor: ['read', 'write'],
  viewer: ['read']
};

const authorize = (requiredPermission) => (req, res, next) => {
  const userPermissions = permissions[req.user.role] || [];
  if (!userPermissions.includes(requiredPermission)) {
    return res.status(403).json({ error: 'Forbidden' });
  }
  next();
};
```
