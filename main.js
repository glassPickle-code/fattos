/* ─────────────────────────────────────────────────────
   FattOs Website — main.js
───────────────────────────────────────────────────── */

/* ── NAV: scroll state ────────────────────────────── */
const nav = document.getElementById('nav');
const onScroll = () => {
    nav.classList.toggle('scrolled', window.scrollY > 20);
};
window.addEventListener('scroll', onScroll, { passive: true });
onScroll();

/* ── NAV: hamburger ───────────────────────────────── */
const hamburger = document.getElementById('navHamburger');
const navLinks  = document.querySelector('.nav-links');

hamburger.addEventListener('click', () => {
    const expanded = hamburger.getAttribute('aria-expanded') === 'true';
    hamburger.setAttribute('aria-expanded', String(!expanded));
    navLinks.classList.toggle('open', !expanded);
});

// close on link click (mobile)
navLinks.querySelectorAll('a').forEach(link => {
    link.addEventListener('click', () => {
        hamburger.setAttribute('aria-expanded', 'false');
        navLinks.classList.remove('open');
    });
});

/* ── SCROLL REVEAL ────────────────────────────────── */
const revealObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting) {
            entry.target.classList.add('visible');
            revealObserver.unobserve(entry.target);
        }
    });
}, {
    threshold: 0.08,
    rootMargin: '0px 0px -40px 0px'
});

document.querySelectorAll('.reveal').forEach(el => {
    revealObserver.observe(el);
});

/* ── TOAST ────────────────────────────────────────── */
let toastTimer = null;

function showNotice() {
    const toast = document.getElementById('infoNotice');
    toast.removeAttribute('hidden');
    // re-trigger animation
    toast.style.animation = 'none';
    // force reflow
    void toast.offsetHeight;
    toast.style.animation = '';

    clearTimeout(toastTimer);
    toastTimer = setTimeout(hideNotice, 5000);
}

function hideNotice() {
    const toast = document.getElementById('infoNotice');
    toast.setAttribute('hidden', '');
    clearTimeout(toastTimer);
}

// Expose to HTML onclick
window.showNotice = showNotice;
window.hideNotice = hideNotice;

/* ── HERO LOGO: subtle float ──────────────────────── */
const heroLogo = document.querySelector('.hero-logo');
if (heroLogo && !window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    let frame;
    const floatLogo = () => {
        const t = performance.now() / 1000;
        const y = Math.sin(t * 0.65) * 5;
        heroLogo.style.transform = `translateY(${y}px)`;
        frame = requestAnimationFrame(floatLogo);
    };
    frame = requestAnimationFrame(floatLogo);
}

/* ── FEATURE CARDS: magnetic micro-tilt ──────────── */
if (!window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
    document.querySelectorAll('.feature-card').forEach(card => {
        card.addEventListener('mousemove', (e) => {
            const r = card.getBoundingClientRect();
            const x = (e.clientX - r.left) / r.width  - 0.5;
            const y = (e.clientY - r.top)  / r.height - 0.5;
            card.style.transform = `translateY(-4px) rotateY(${x * 6}deg) rotateX(${-y * 4}deg)`;
        });
        card.addEventListener('mouseleave', () => {
            card.style.transform = '';
        });
    });
}

/* ── NAV ACTIVE LINK HIGHLIGHT ────────────────────── */
const sections = document.querySelectorAll('section[id], header[id]');
const links    = document.querySelectorAll('.nav-links a[href^="#"]');

const sectionObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting) {
            const id = entry.target.getAttribute('id');
            links.forEach(link => {
                const active = link.getAttribute('href') === `#${id}`;
                link.style.color   = active ? 'var(--dark)' : '';
                link.style.background = active ? 'var(--tan-dim)' : '';
            });
        }
    });
}, { threshold: 0.45 });

sections.forEach(s => sectionObserver.observe(s));
