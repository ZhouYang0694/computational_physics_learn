use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

/// Provides a per-simulation directory under `output/`.
pub struct RunContext {
    root: PathBuf,
}

impl RunContext {
    /// Creates (or reuses) `output/<name>` and returns the context.
    pub fn new(name: &str) -> std::io::Result<Self> {
        let root = PathBuf::from("output").join(name);
        fs::create_dir_all(&root)?;
        Ok(Self { root })
    }

    /// Returns the path of a file inside the context directory.
    pub fn file_path(&self, name: &str) -> PathBuf {
        self.root.join(name)
    }

    /// Writes plain text into the provided relative path.
    pub fn write_text(&self, name: &str, contents: &str) -> std::io::Result<PathBuf> {
        let path = self.file_path(name);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let mut file = File::create(&path)?;
        file.write_all(contents.as_bytes())?;
        Ok(path)
    }

    /// Writes multiple lines joined by newlines.
    pub fn write_lines(&self, name: &str, lines: &[String]) -> std::io::Result<PathBuf> {
        let mut text = String::new();
        for (idx, line) in lines.iter().enumerate() {
            if idx > 0 {
                text.push('\n');
            }
            text.push_str(line);
        }
        self.write_text(name, &text)
    }

    /// Writes a CSV file constructed from headers and rows.
    pub fn write_csv(
        &self,
        name: &str,
        headers: &[&str],
        rows: &[Vec<String>],
    ) -> std::io::Result<PathBuf> {
        let path = self.file_path(name);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let file = File::create(&path)?;
        let mut writer = BufWriter::new(file);
        write_csv_row(&mut writer, headers)?;
        for row in rows {
            let cols: Vec<&str> = row.iter().map(|s| s.as_str()).collect();
            write_csv_row(&mut writer, &cols)?;
        }
        writer.flush()?;
        Ok(path)
    }

    pub fn root(&self) -> &Path {
        &self.root
    }
}

fn write_csv_row(writer: &mut BufWriter<File>, columns: &[&str]) -> std::io::Result<()> {
    for (idx, value) in columns.iter().enumerate() {
        if idx > 0 {
            writer.write_all(b",")?;
        }
        if value.contains(',') || value.contains('"') || value.contains('\n') {
            writer.write_all(b"\"")?;
            for ch in value.chars() {
                if ch == '"' {
                    writer.write_all(b"\"")?;
                }
                let mut buf = [0u8; 4];
                let encoded = ch.encode_utf8(&mut buf);
                writer.write_all(encoded.as_bytes())?;
            }
            writer.write_all(b"\"")?;
        } else {
            writer.write_all(value.as_bytes())?;
        }
    }
    writer.write_all(b"\n")?;
    Ok(())
}
