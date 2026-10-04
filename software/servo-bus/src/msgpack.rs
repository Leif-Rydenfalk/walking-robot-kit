//! The slice of MessagePack the Arduino RouterBridge speaks: nil, bool, int, float, str, bin,
//! array, map. Requests are `[0, msgid, method, params]`, replies `[1, msgid, error, result]`.

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Nil,
    Bool(bool),
    Int(i64),
    F64(f64),
    Str(String),
    Bin(Vec<u8>),
    Arr(Vec<Value>),
    Map(Vec<(Value, Value)>),
}

impl Value {
    pub fn as_i64(&self) -> Option<i64> {
        match self {
            Value::Int(i) => Some(*i),
            _ => None,
        }
    }
    /// A byte vector the MCU returned: `bin` from `std::vector<uint8_t>`, or an int array.
    pub fn as_bytes(&self) -> Option<Vec<u8>> {
        match self {
            Value::Bin(b) => Some(b.clone()),
            Value::Str(s) => Some(s.as_bytes().to_vec()),
            Value::Arr(a) => a.iter().map(|v| v.as_i64().map(|i| i as u8)).collect(),
            _ => None,
        }
    }
}

pub fn encode(v: &Value, out: &mut Vec<u8>) {
    match v {
        Value::Nil => out.push(0xc0),
        Value::Bool(b) => out.push(if *b { 0xc3 } else { 0xc2 }),
        Value::Int(i) => enc_int(*i, out),
        Value::F64(f) => {
            out.push(0xcb);
            out.extend_from_slice(&f.to_be_bytes());
        }
        Value::Str(s) => {
            let n = s.len();
            if n < 32 {
                out.push(0xa0 | n as u8);
            } else if n < 256 {
                out.extend_from_slice(&[0xd9, n as u8]);
            } else if n < 65536 {
                out.push(0xda);
                out.extend_from_slice(&(n as u16).to_be_bytes());
            } else {
                out.push(0xdb);
                out.extend_from_slice(&(n as u32).to_be_bytes());
            }
            out.extend_from_slice(s.as_bytes());
        }
        Value::Bin(b) => {
            let n = b.len();
            if n < 256 {
                out.extend_from_slice(&[0xc4, n as u8]);
            } else if n < 65536 {
                out.push(0xc5);
                out.extend_from_slice(&(n as u16).to_be_bytes());
            } else {
                out.push(0xc6);
                out.extend_from_slice(&(n as u32).to_be_bytes());
            }
            out.extend_from_slice(b);
        }
        Value::Arr(a) => {
            let n = a.len();
            if n < 16 {
                out.push(0x90 | n as u8);
            } else if n < 65536 {
                out.push(0xdc);
                out.extend_from_slice(&(n as u16).to_be_bytes());
            } else {
                out.push(0xdd);
                out.extend_from_slice(&(n as u32).to_be_bytes());
            }
            for x in a {
                encode(x, out);
            }
        }
        Value::Map(m) => {
            let n = m.len();
            if n < 16 {
                out.push(0x80 | n as u8);
            } else {
                out.push(0xde);
                out.extend_from_slice(&(n as u16).to_be_bytes());
            }
            for (k, x) in m {
                encode(k, out);
                encode(x, out);
            }
        }
    }
}

fn enc_int(i: i64, out: &mut Vec<u8>) {
    if (0..128).contains(&i) {
        out.push(i as u8);
    } else if (-32..0).contains(&i) {
        out.push(i as i8 as u8);
    } else if i >= 0 && i <= u8::MAX as i64 {
        out.extend_from_slice(&[0xcc, i as u8]);
    } else if i >= 0 && i <= u16::MAX as i64 {
        out.push(0xcd);
        out.extend_from_slice(&(i as u16).to_be_bytes());
    } else if i >= 0 && i <= u32::MAX as i64 {
        out.push(0xce);
        out.extend_from_slice(&(i as u32).to_be_bytes());
    } else if i >= i32::MIN as i64 && i <= i32::MAX as i64 {
        out.push(0xd2);
        out.extend_from_slice(&(i as i32).to_be_bytes());
    } else {
        out.push(0xd3);
        out.extend_from_slice(&i.to_be_bytes());
    }
}

/// Decode one value from the front of `b`. `None` means "not all bytes are here yet".
/// Every type byte decodes to something, so a well-framed stream never errors.
pub fn decode(b: &[u8]) -> Option<(Value, usize)> {
    let mut p = 0usize;
    dec(b, &mut p).ok().map(|v| (v, p))
}

struct Short;

fn take<'a>(b: &'a [u8], p: &mut usize, n: usize) -> Result<&'a [u8], Short> {
    if *p + n > b.len() {
        return Err(Short);
    }
    let s = &b[*p..*p + n];
    *p += n;
    Ok(s)
}

fn be(s: &[u8]) -> u64 {
    s.iter().fold(0u64, |a, &x| a << 8 | x as u64)
}

fn dec(b: &[u8], p: &mut usize) -> Result<Value, Short> {
    let t = take(b, p, 1)?[0];
    Ok(match t {
        0x00..=0x7f => Value::Int(t as i64),
        0x80..=0x8f => dec_map(b, p, (t & 0x0f) as usize)?,
        0x90..=0x9f => dec_arr(b, p, (t & 0x0f) as usize)?,
        0xa0..=0xbf => dec_str(b, p, (t & 0x1f) as usize)?,
        0xc0 => Value::Nil,
        0xc2 => Value::Bool(false),
        0xc3 => Value::Bool(true),
        0xc4 => {
            let n = take(b, p, 1)?[0] as usize;
            Value::Bin(take(b, p, n)?.to_vec())
        }
        0xc5 => {
            let n = be(take(b, p, 2)?) as usize;
            Value::Bin(take(b, p, n)?.to_vec())
        }
        0xc6 => {
            let n = be(take(b, p, 4)?) as usize;
            Value::Bin(take(b, p, n)?.to_vec())
        }
        0xca => Value::F64(f32::from_bits(be(take(b, p, 4)?) as u32) as f64),
        0xcb => Value::F64(f64::from_bits(be(take(b, p, 8)?))),
        0xcc => Value::Int(be(take(b, p, 1)?) as i64),
        0xcd => Value::Int(be(take(b, p, 2)?) as i64),
        0xce => Value::Int(be(take(b, p, 4)?) as i64),
        0xcf => Value::Int(be(take(b, p, 8)?) as i64),
        0xd0 => Value::Int(take(b, p, 1)?[0] as i8 as i64),
        0xd1 => Value::Int(be(take(b, p, 2)?) as u16 as i16 as i64),
        0xd2 => Value::Int(be(take(b, p, 4)?) as u32 as i32 as i64),
        0xd3 => Value::Int(be(take(b, p, 8)?) as i64),
        0xd9 => {
            let n = take(b, p, 1)?[0] as usize;
            dec_str(b, p, n)?
        }
        0xda => {
            let n = be(take(b, p, 2)?) as usize;
            dec_str(b, p, n)?
        }
        0xdb => {
            let n = be(take(b, p, 4)?) as usize;
            dec_str(b, p, n)?
        }
        0xdc => {
            let n = be(take(b, p, 2)?) as usize;
            dec_arr(b, p, n)?
        }
        0xdd => {
            let n = be(take(b, p, 4)?) as usize;
            dec_arr(b, p, n)?
        }
        0xde => {
            let n = be(take(b, p, 2)?) as usize;
            dec_map(b, p, n)?
        }
        0xdf => {
            let n = be(take(b, p, 4)?) as usize;
            dec_map(b, p, n)?
        }
        0xe0..=0xff => Value::Int(t as i8 as i64),
        // ext types: skip them as Nil so one odd frame cannot wedge the stream
        0xd4 => skip(b, p, 2)?,
        0xd5 => skip(b, p, 3)?,
        0xd6 => skip(b, p, 5)?,
        0xd7 => skip(b, p, 9)?,
        0xd8 => skip(b, p, 17)?,
        0xc7 => {
            let n = take(b, p, 1)?[0] as usize;
            skip(b, p, n + 1)?
        }
        0xc8 => {
            let n = be(take(b, p, 2)?) as usize;
            skip(b, p, n + 1)?
        }
        0xc9 => {
            let n = be(take(b, p, 4)?) as usize;
            skip(b, p, n + 1)?
        }
        0xc1 => Value::Nil,
    })
}

fn skip(b: &[u8], p: &mut usize, n: usize) -> Result<Value, Short> {
    take(b, p, n)?;
    Ok(Value::Nil)
}

fn dec_str(b: &[u8], p: &mut usize, n: usize) -> Result<Value, Short> {
    Ok(Value::Str(String::from_utf8_lossy(take(b, p, n)?).into_owned()))
}

fn dec_arr(b: &[u8], p: &mut usize, n: usize) -> Result<Value, Short> {
    let mut v = Vec::with_capacity(n.min(1024));
    for _ in 0..n {
        v.push(dec(b, p)?);
    }
    Ok(Value::Arr(v))
}

fn dec_map(b: &[u8], p: &mut usize, n: usize) -> Result<Value, Short> {
    let mut v = Vec::with_capacity(n.min(1024));
    for _ in 0..n {
        let k = dec(b, p)?;
        let x = dec(b, p)?;
        v.push((k, x));
    }
    Ok(Value::Map(v))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let v = Value::Arr(vec![
            Value::Int(0),
            Value::Int(70000),
            Value::Str("bus_batch".into()),
            Value::Arr(vec![Value::Bin(vec![1, 2, 255]), Value::Int(-5), Value::Int(3000)]),
            Value::Nil,
            Value::Bool(true),
        ]);
        let mut b = Vec::new();
        encode(&v, &mut b);
        let (back, n) = decode(&b).unwrap();
        assert_eq!(n, b.len());
        assert_eq!(back, v);
        // every strict prefix is "not yet", never an error or a wrong value
        for k in 0..b.len() {
            assert!(decode(&b[..k]).is_none(), "prefix {k}");
        }
    }

    #[test]
    fn bytes_from_bin_or_array() {
        assert_eq!(Value::Bin(vec![9, 8]).as_bytes(), Some(vec![9, 8]));
        assert_eq!(Value::Arr(vec![Value::Int(9), Value::Int(255)]).as_bytes(), Some(vec![9, 255]));
    }
}
