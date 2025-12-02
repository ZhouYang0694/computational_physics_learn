#[derive(Debug, Clone, Copy)]
pub struct LinearFit {
    pub slope: f64,
    pub intercept: f64,
    pub r2: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ExponentialFit {
    pub n0: f64,
    pub tau: f64,
    pub r2: f64,
}

pub fn linear_regression(x: &[f64], y: &[f64]) -> Option<LinearFit> {
    if x.len() != y.len() || x.len() < 2 {
        return None;
    }
    let n = x.len() as f64;
    let mean_x = x.iter().sum::<f64>() / n;
    let mean_y = y.iter().sum::<f64>() / n;

    let mut ss_xx = 0.0;
    let mut ss_xy = 0.0;
    let mut ss_tot = 0.0;
    let mut ss_res = 0.0;

    for (&xi, &yi) in x.iter().zip(y.iter()) {
        let dx = xi - mean_x;
        let dy = yi - mean_y;
        ss_xx += dx * dx;
        ss_xy += dx * dy;
    }

    if ss_xx.abs() < f64::EPSILON {
        return None;
    }

    let slope = ss_xy / ss_xx;
    let intercept = mean_y - slope * mean_x;

    for (&xi, &yi) in x.iter().zip(y.iter()) {
        let y_hat = slope * xi + intercept;
        let residual = yi - y_hat;
        ss_res += residual * residual;
        let dy = yi - mean_y;
        ss_tot += dy * dy;
    }

    let r2 = if ss_tot.abs() < f64::EPSILON {
        1.0
    } else {
        1.0 - (ss_res / ss_tot)
    };

    Some(LinearFit {
        slope,
        intercept,
        r2,
    })
}

pub fn exponential_fit(t: &[f64], n: &[f64]) -> Option<ExponentialFit> {
    if t.len() != n.len() {
        return None;
    }

    let mut x = Vec::new();
    let mut y = Vec::new();
    for (&ti, &ni) in t.iter().zip(n.iter()) {
        if ni > 0.0 {
            x.push(ti);
            y.push(ni.ln());
        }
    }

    let fit = linear_regression(&x, &y)?;
    if fit.slope >= 0.0 {
        return None;
    }
    let tau = -1.0 / fit.slope;
    let n0 = fit.intercept.exp();

    Some(ExponentialFit {
        n0,
        tau,
        r2: fit.r2,
    })
}
