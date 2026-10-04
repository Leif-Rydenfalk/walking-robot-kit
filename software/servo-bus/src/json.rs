//! The slice of JSON this app needs: page commands in, state and mesh payloads out.
//! Numbers are f64; dumps print integral floats without the decimal point so the page
//! never sees "2048.0".

#[derive(Debug, Clone, PartialEq)]
pub enum J {
    Nil,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<J>),
    Obj(Vec<(String, J)>),
}

impl J {
    pub fn n(v: f64) -> J {
        J::Num(v)
    }
    pub fn s(v: &str) -> J {
        J::Str(v.to_string())
    }
    pub fn obj(pairs: Vec<(&str, J)>) -> J {
        J::Obj(pairs.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
    }
    pub fn get(&self, k: &str) -> Option<&J> {
        match self {
            J::Obj(m) => m.iter().find(|(key, _)| key == k).map(|(_, v)| v),
            _ => None,
        }
    }
    pub fn num(&self) -> Option<f64> {
        match self {
            J::Num(x) => Some(*x),
            _ => None,
        }
    }
    pub fn i64(&self) -> Option<i64> {
        self.num().map(|x| x as i64)
    }
    pub fn boolean(&self) -> Option<bool> {
        match self {
            J::Bool(b) => Some(*b),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            J::Str(s) => Some(s),
            _ => None,
        }
    }
}

pub fn dump(v: &J) -> String {
    let mut out = String::with_capacity(256);
    write_j(v, &mut out);
    out
}

fn write_j(v: &J, out: &mut String) {
    match v {
        J::Nil => out.push_str("null"),
        J::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        J::Num(x) => {
            if x.is_finite() {
                if x.fract() == 0.0 && x.abs() < 9e15 {
                    out.push_str(&format!("{}", *x as i64));
                } else {
                    out.push_str(&format!("{}", x));
                }
            } else {
                out.push_str("null"); // NaN (offline servo in the chart) dumps as null
            }
        }
        J::Str(s) => write_str(s, out),
        J::Arr(a) => {
            out.push('[');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_j(x, out);
            }
            out.push(']');
        }
        J::Obj(m) => {
            out.push('{');
            for (i, (k, x)) in m.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_str(k, out);
                out.push(':');
                write_j(x, out);
            }
            out.push('}');
        }
    }
}

fn write_str(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

/// Parse one JSON document. `None` on anything malformed; the page and mesh only ever
/// send small objects, so a rejected command is logged and dropped, never half-applied.
pub fn parse(s: &str) -> Option<J> {
    let b = s.as_bytes();
    let mut p = 0usize;
    ws(b, &mut p);
    let v = pval(b, &mut p)?;
    ws(b, &mut p);
    if p != b.len() {
        return None;
    }
    Some(v)
}

fn ws(b: &[u8], p: &mut usize) {
    while *p < b.len() && (b[*p] == b' ' || b[*p] == b'\t' || b[*p] == b'\n' || b[*p] == b'\r') {
        *p += 1;
    }
}

fn lit(b: &[u8], p: &mut usize, word: &str) -> bool {
    if b[*p..].starts_with(word.as_bytes()) {
        *p += word.len();
        true
    } else {
        false
    }
}

fn pval(b: &[u8], p: &mut usize) -> Option<J> {
    ws(b, p);
    if *p >= b.len() {
        return None;
    }
    match b[*p] {
        b'{' => {
            *p += 1;
            let mut m = Vec::new();
            ws(b, p);
            if *p < b.len() && b[*p] == b'}' {
                *p += 1;
                return Some(J::Obj(m));
            }
            loop {
                ws(b, p);
                let k = pstr(b, p)?;
                ws(b, p);
                if *p >= b.len() || b[*p] != b':' {
                    return None;
                }
                *p += 1;
                let v = pval(b, p)?;
                m.push((k, v));
                ws(b, p);
                match b.get(*p)? {
                    b',' => *p += 1,
                    b'}' => {
                        *p += 1;
                        return Some(J::Obj(m));
                    }
                    _ => return None,
                }
            }
        }
        b'[' => {
            *p += 1;
            let mut a = Vec::new();
            ws(b, p);
            if *p < b.len() && b[*p] == b']' {
                *p += 1;
                return Some(J::Arr(a));
            }
            loop {
                let v = pval(b, p)?;
                a.push(v);
                ws(b, p);
                match b.get(*p)? {
                    b',' => *p += 1,
                    b']' => {
                        *p += 1;
                        return Some(J::Arr(a));
                    }
                    _ => return None,
                }
            }
        }
        b'"' => pstr(b, p).map(J::Str),
        b't' => lit(b, p, "true").then_some(J::Bool(true)),
        b'f' => lit(b, p, "false").then_some(J::Bool(false)),
        b'n' => lit(b, p, "null").then_some(J::Nil),
        b'-' | b'0'..=b'9' => pnum(b, p),
        _ => None,
    }
}

fn pstr(b: &[u8], p: &mut usize) -> Option<String> {
    if b.get(*p)? != &b'"' {
        return None;
    }
    *p += 1;
    let mut out = String::new();
    loop {
        let c = *b.get(*p)?;
        *p += 1;
        match c {
            b'"' => return Some(out),
            b'\\' => {
                let e = *b.get(*p)?;
                *p += 1;
                match e {
                    b'"' => out.push('"'),
                    b'\\' => out.push('\\'),
                    b'/' => out.push('/'),
                    b'n' => out.push('\n'),
                    b'r' => out.push('\r'),
                    b't' => out.push('\t'),
                    b'b' => out.push('\u{8}'),
                    b'f' => out.push('\u{c}'),
                    b'u' => {
                        let hex = std::str::from_utf8(b.get(*p..*p + 4)?).ok()?;
                        let cp = u32::from_str_radix(hex, 16).ok()?;
                        *p += 4;
                        out.push(char::from_u32(cp).unwrap_or('\u{fffd}'));
                    }
                    _ => return None,
                }
            }
            c => out.push(c as char), // the page sends ASCII commands only
        }
    }
}

fn pnum(b: &[u8], p: &mut usize) -> Option<J> {
    let start = *p;
    if b.get(*p) == Some(&b'-') {
        *p += 1;
    }
    while matches!(b.get(*p), Some(c) if c.is_ascii_digit()) {
        *p += 1;
    }
    if b.get(*p) == Some(&b'.') {
        *p += 1;
        while matches!(b.get(*p), Some(c) if c.is_ascii_digit()) {
            *p += 1;
        }
    }
    if matches!(b.get(*p), Some(b'e') | Some(b'E')) {
        *p += 1;
        if matches!(b.get(*p), Some(b'+') | Some(b'-')) {
            *p += 1;
        }
        while matches!(b.get(*p), Some(c) if c.is_ascii_digit()) {
            *p += 1;
        }
    }
    if *p == start {
        return None;
    }
    std::str::from_utf8(&b[start..*p]).ok()?.parse::<f64>().ok().map(J::Num)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_page_command() {
        let v = parse(r#"{"op":"move","id":2,"deg":45.5}"#).unwrap();
        assert_eq!(v.get("op").unwrap().as_str(), Some("move"));
        assert_eq!(v.get("id").unwrap().i64(), Some(2));
        let mut d = String::new();
        write_j(&J::n(2048.0), &mut d);
        assert_eq!(d, "2048");
    }

    #[test]
    fn rejects_garbage() {
        assert!(parse("{op:1}").is_none());
        assert!(parse("{\"a\":1}x").is_none());
        assert!(parse("{\"a\":}").is_none());
        assert!(parse("").is_none());
    }

    #[test]
    fn escapes_and_nan() {
        let v = J::obj(vec![("s", J::s("a\"b\n"))]);
        assert_eq!(dump(&v), r#"{"s":"a\"b\n"}"#);
        assert_eq!(dump(&J::n(f64::NAN)), "null");
        assert_eq!(dump(&J::n(3.5)), "3.5");
    }
}
