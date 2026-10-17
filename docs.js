/* =======================================================
   FattOs Documentation Viewer - docs.js
   Dynamic Markdown Loader, GFM Parser & Interactive Reader
======================================================= */

(function () {
    // Registered docs catalog: filename, display title (without .md), description, category
    const DOCS_CATALOG = [
        {
            id: 'overview',
            file: 'overview.md',
            title: 'Overview',
            category: 'Introduction',
            desc: 'High-level introduction to FattOs, core layers, supervised scheduler & robotics stack'
        },
        {
            id: 'getting-started',
            file: 'getting-started.md',
            title: 'Getting Started',
            category: 'Guides',
            desc: 'Zero-to-running guide: building library, sensor manifest, control loop, and host demo'
        },
        {
            id: 'architecture',
            file: 'architecture.md',
            title: 'Architecture',
            category: 'Core System',
            desc: 'System layout, zero-heap memory philosophy, execution models, and isolation layers'
        },
        {
            id: 'kernel',
            file: 'kernel.md',
            title: 'Kernel & Scheduler',
            category: 'Core System',
            desc: 'Cooperative scheduler, fair dispatch, deadline monitoring, task profiling & timers'
        },
        {
            id: 'sensors',
            file: 'sensors.md',
            title: 'Sensors & Manifest',
            category: 'Hardware',
            desc: 'Static hardware manifest, channel calibration, sensor fusion, and driver interfaces'
        },
        {
            id: 'edge-ai',
            file: 'edge-ai.md',
            title: 'Edge AI & Neural Net',
            category: 'Intelligence',
            desc: 'Native INT8 neural network runtime, static tensor arena, and feature extraction pipeline'
        },
        {
            id: 'autonomy',
            file: 'autonomy.md',
            title: 'Autonomy & Navigation',
            category: 'Robotics',
            desc: 'State machine, flight-controller arming semantics, go-to-goal navigation & safety estop'
        },
        {
            id: 'porting-stm32',
            file: 'porting-stm32.md',
            title: 'Porting to STM32',
            category: 'Hardware',
            desc: 'Porting guide for STM32F4 and STM32F7+ microcontrollers, clock setup & timers'
        }
    ];

    // State
    let currentDocId = 'getting-started';
    const GITHUB_REPO_BASE = 'https://github.com/glassPickle-code/fattos';

    // DOM Elements
    const docsListEl = document.getElementById('docsList');
    const searchInput = document.getElementById('docsSearch');
    const docTitleEl = document.getElementById('docTitle');
    const breadcrumbDocEl = document.getElementById('breadcrumbDoc');
    const markdownContentEl = document.getElementById('markdownContent');
    const readTimeEl = document.getElementById('readTime');
    const rawGithubLinkEl = document.getElementById('rawGithubLink');
    const tocListEl = document.getElementById('tocList');
    const paginationPrevEl = document.getElementById('paginationPrev');
    const paginationNextEl = document.getElementById('paginationNext');
    const mobileToggleBtn = document.getElementById('mobileSidebarToggle');
    const sidebarEl = document.getElementById('docsSidebar');

    // Initialize list of docs (without .md visible)
    function renderDocsList(filterText = '') {
        docsListEl.innerHTML = '';
        const normFilter = filterText.toLowerCase().trim();

        const filtered = DOCS_CATALOG.filter(doc => {
            return doc.title.toLowerCase().includes(normFilter) ||
                   doc.desc.toLowerCase().includes(normFilter) ||
                   doc.category.toLowerCase().includes(normFilter);
        });

        if (filtered.length === 0) {
            const empty = document.createElement('li');
            empty.style.padding = '0.75rem';
            empty.style.fontSize = '0.8rem';
            empty.style.color = 'var(--muted)';
            empty.textContent = 'No matching documents found';
            docsListEl.appendChild(empty);
            return;
        }

        filtered.forEach(doc => {
            const li = document.createElement('li');
            const btn = document.createElement('button');
            btn.className = `docs-item-btn ${doc.id === currentDocId ? 'active' : ''}`;
            btn.setAttribute('data-id', doc.id);
            btn.innerHTML = `
                <svg class="docs-item-icon" viewBox="0 0 20 20" fill="currentColor">
                    <path fill-rule="evenodd" d="M4 4a2 2 0 012-2h4.586A2 2 0 0112 2.586L15.414 6A2 2 0 0116 7.414V16a2 2 0 01-2 2H6a2 2 0 01-2-2V4zm2 6a1 1 0 011-1h6a1 1 0 110 2H7a1 1 0 01-1-1zm1 3a1 1 0 100 2h6a1 1 0 100-2H7z" clip-rule="evenodd"/>
                </svg>
                <span>${escapeHtml(doc.title)}</span>
            `;

            btn.addEventListener('click', () => {
                loadDocument(doc.id);
                if (window.innerWidth <= 860) {
                    sidebarEl.classList.remove('open');
                }
            });

            li.appendChild(btn);
            docsListEl.appendChild(li);
        });
    }

    // Markdown Parser
    function parseMarkdown(md) {
        if (!md) return '';

        // Normalize line breaks
        let text = md.replace(/\r\n/g, '\n');

        // Extract and protect code blocks
        const codeBlocks = [];
        text = text.replace(/```([a-zA-Z0-9_-]*)\n([\s\S]*?)```/g, (match, lang, code) => {
            const index = codeBlocks.length;
            codeBlocks.push({ lang: lang.trim().toLowerCase(), code });
            return `%%CODE_BLOCK_${index}%%`;
        });

        // Blockquotes & Notes (> [!NOTE] or > text)
        text = text.replace(/^>\s*(.*)$/gm, '<blockquote><p>$1</p></blockquote>');
        text = text.replace(/<\/blockquote>\n<blockquote>/g, ''); // merge adjacent blockquotes

        // Headings (# h1, ## h2, ### h3, #### h4)
        text = text.replace(/^#### (.*$)/gm, (match, title) => {
            const id = slugify(title);
            return `<h4 id="${id}">${escapeHtml(title)}</h4>`;
        });
        text = text.replace(/^### (.*$)/gm, (match, title) => {
            const id = slugify(title);
            return `<h3 id="${id}">${escapeHtml(title)}</h3>`;
        });
        text = text.replace(/^## (.*$)/gm, (match, title) => {
            const id = slugify(title);
            return `<h2 id="${id}">${escapeHtml(title)}</h2>`;
        });
        text = text.replace(/^# (.*$)/gm, (match, title) => {
            const id = slugify(title);
            return `<h1 id="${id}">${escapeHtml(title)}</h1>`;
        });

        // Horizontal Rule
        text = text.replace(/^---$/gm, '<hr>');

        // Tables (GFM)
        text = text.replace(/((?:\|[^\n]+\|\n?)+)/g, (match) => {
            const rows = match.trim().split('\n').filter(r => r.trim());
            if (rows.length < 2) return match;

            let html = '<table>';
            let isHeader = true;

            rows.forEach((row) => {
                if (/^\|?\s*[-:]+[-| :]*\|?$/.test(row)) {
                    isHeader = false;
                    return;
                }
                const cells = row.split('|').slice(1, -1);
                if (isHeader) {
                    html += '<thead><tr>';
                    cells.forEach(c => { html += `<th>${c.trim()}</th>`; });
                    html += '</tr></thead><tbody>';
                } else {
                    html += '<tr>';
                    cells.forEach(c => { html += `<td>${c.trim()}</td>`; });
                    html += '</tr>';
                }
            });
            html += '</tbody></table>';
            return html;
        });

        // Inline formatting
        // Bold
        text = text.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
        // Italic
        text = text.replace(/(^|[^*])\*([^*]+)\*/g, '$1<em>$2</em>');
        text = text.replace(/(^|[^_])_([^_]+)_/g, '$1<em>$2</em>');
        // Inline code
        text = text.replace(/`([^`]+)`/g, (m, c) => `<code>${escapeHtml(c)}</code>`);

        // Links: [text](url)
        text = text.replace(/\[([^\]]+)\]\(([^)]+)\)/g, (match, text, url) => {
            const target = url.startsWith('http') ? ' target="_blank" rel="noopener"' : '';
            return `<a href="${url}"${target}>${text}</a>`;
        });

        // Lists
        const lines = text.split('\n');
        let inUl = false;
        let inOl = false;
        const result = [];

        lines.forEach(line => {
            const ulMatch = line.match(/^(\s*)[-*+]\s+(.*)$/);
            const olMatch = line.match(/^(\s*)\d+\.\s+(.*)$/);

            if (ulMatch) {
                if (inOl) { result.push('</ol>'); inOl = false; }
                if (!inUl) { result.push('<ul>'); inUl = true; }
                result.push(`<li>${ulMatch[2]}</li>`);
            } else if (olMatch) {
                if (inUl) { result.push('</ul>'); inUl = false; }
                if (!inOl) { result.push('<ol>'); inOl = true; }
                result.push(`<li>${olMatch[2]}</li>`);
            } else {
                if (inUl) { result.push('</ul>'); inUl = false; }
                if (inOl) { result.push('</ol>'); inOl = false; }
                if (line.trim() && !line.startsWith('<h') && !line.startsWith('<table') && !line.startsWith('<tbody') && !line.startsWith('<tr') && !line.startsWith('<td') && !line.startsWith('<th') && !line.startsWith('</table') && !line.startsWith('<hr') && !line.startsWith('<blockquote') && !line.startsWith('%%CODE_BLOCK_')) {
                    result.push(`<p>${line}</p>`);
                } else {
                    result.push(line);
                }
            }
        });

        if (inUl) result.push('</ul>');
        if (inOl) result.push('</ol>');

        text = result.join('\n');

        // Restore code blocks with syntax highlighting & copy button
        text = text.replace(/%%CODE_BLOCK_(\d+)%%/g, (match, index) => {
            const block = codeBlocks[Number(index)];
            const rawCode = block.code;
            const highlighted = highlightCode(rawCode, block.lang);
            const langLabel = block.lang || 'code';

            return `
            <div class="code-block-wrapper">
                <div class="code-block-header">
                    <span class="code-lang-tag">${escapeHtml(langLabel)}</span>
                    <button class="code-copy-btn" onclick="copySnippet(this)" data-code="${escapeAttribute(rawCode)}">
                        <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2" ry="2"/><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/></svg>
                        <span>Copy</span>
                    </button>
                </div>
                <pre><code class="language-${langLabel}">${highlighted}</code></pre>
            </div>`;
        });

        return text;
    }

    // Lightweight Syntax Highlighter for C, Bash, CMake, JSON
    function highlightCode(code, lang) {
        let escaped = escapeHtml(code);

        if (lang === 'c' || lang === 'cpp' || lang === 'h') {
            // Strings
            escaped = escaped.replace(/(&quot;.*?&quot;)/g, '<span class="c-str">$1</span>');
            // Comments
            escaped = escaped.replace(/(\/\*[\s\S]*?\*\/|\/\/[^\n]*)/g, '<span class="c-cm">$1</span>');
            // Keywords
            escaped = escaped.replace(/\b(void|int|char|float|double|uint32_t|uint16_t|uint8_t|int32_t|int16_t|int8_t|bool|size_t|struct|typedef|static|const|return|if|else|for|while|switch|case|break|continue|default)\b/g, '<span class="c-kw">$1</span>');
            // Functions
            escaped = escaped.replace(/\b([a-zA-Z_][a-zA-Z0-9_]*)(?=\()/g, '<span class="c-fn">$1</span>');
            // Numbers
            escaped = escaped.replace(/\b(0x[0-9a-fA-F]+|\d+)\b/g, '<span class="c-num">$1</span>');
        } else if (lang === 'sh' || lang === 'bash' || lang === 'shell' || lang === 'cmake') {
            // Comments
            escaped = escaped.replace(/(#[^\n]*)/g, '<span class="c-cm">$1</span>');
            // Commands / Keywords
            escaped = escaped.replace(/\b(cmake|ctest|make|git|gcc|clang|cd|mkdir|cp|run|curl)\b/g, '<span class="c-kw">$1</span>');
            // Options / flags
            escaped = escaped.replace(/(-[a-zA-Z0-9_-]+)/g, '<span class="c-type">$1</span>');
        }

        return escaped;
    }

    // Utilities
    function escapeHtml(str) {
        if (!str) return '';
        return str
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;')
            .replace(/'/g, '&#39;');
    }

    function escapeAttribute(str) {
        if (!str) return '';
        return str
            .replace(/&/g, '&amp;')
            .replace(/"/g, '&quot;')
            .replace(/'/g, '&#39;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;');
    }

    function slugify(text) {
        return text.toString().toLowerCase().trim()
            .replace(/\s+/g, '-')
            .replace(/[^\w\-]+/g, '')
            .replace(/\-\-+/g, '-');
    }

    // Load Document
    async function loadDocument(docId) {
        const docInfo = DOCS_CATALOG.find(d => d.id === docId) || DOCS_CATALOG[0];
        currentDocId = docInfo.id;

        // Update active class in sidebar
        document.querySelectorAll('.docs-item-btn').forEach(btn => {
            btn.classList.toggle('active', btn.getAttribute('data-id') === docInfo.id);
        });

        // Update Title & Breadcrumbs
        docTitleEl.textContent = docInfo.title;
        breadcrumbDocEl.textContent = docInfo.title;
        document.title = `${docInfo.title} — FattOs™ Documentation`;

        // Update Raw GitHub Link
        rawGithubLinkEl.href = `${GITHUB_REPO_BASE}/blob/main/docs/${docInfo.file}`;

        // Fetch or load from fallback bundle
        let markdownText = '';
        try {
            // First attempt: live HTTP fetch from ./docs/
            const response = await fetch(`./docs/${docInfo.file}?t=${Date.now()}`);
            if (response.ok) {
                markdownText = await response.text();
            } else {
                throw new Error(`HTTP ${response.status}`);
            }
        } catch (e) {
            // Second attempt: Fallback to pre-bundled cache in window.FATTOS_DOCS_BUNDLE
            if (window.FATTOS_DOCS_BUNDLE && window.FATTOS_DOCS_BUNDLE[docInfo.file]) {
                markdownText = window.FATTOS_DOCS_BUNDLE[docInfo.file];
            } else {
                markdownText = `# ${docInfo.title}\n\nUnable to load documentation from \`docs/${docInfo.file}\`. Please verify network or file availability.\n\nYou can also read it directly on GitHub at [${docInfo.file}](${GITHUB_REPO_BASE}/blob/main/docs/${docInfo.file}).`;
            }
        }

        // Calculate Reading Time
        const words = markdownText.trim().split(/\s+/).length;
        const readMinutes = Math.max(1, Math.round(words / 200));
        readTimeEl.textContent = `${readMinutes} min read`;

        // Render Markdown
        markdownContentEl.innerHTML = parseMarkdown(markdownText);

        // Build Table of Contents
        buildTableOfContents();

        // Update Pagination (Prev / Next)
        updatePagination(docInfo.id);

        // Update URL hash
        if (window.location.hash.slice(1) !== docInfo.id) {
            history.pushState(null, '', `#${docInfo.id}`);
        }

        // Scroll to top of viewer
        window.scrollTo({ top: 0, behavior: 'smooth' });
    }

    // Build Table of Contents ("On this page")
    function buildTableOfContents() {
        tocListEl.innerHTML = '';
        const headings = markdownContentEl.querySelectorAll('h2, h3');

        if (headings.length === 0) {
            tocListEl.innerHTML = '<li style="color:var(--muted); font-size:0.75rem;">No sections</li>';
            return;
        }

        headings.forEach(heading => {
            const li = document.createElement('li');
            const a = document.createElement('a');
            a.className = `toc-link ${heading.tagName.toLowerCase()}`;
            a.href = `#${heading.id}`;
            a.textContent = heading.textContent;
            a.addEventListener('click', (e) => {
                e.preventDefault();
                heading.scrollIntoView({ behavior: 'smooth' });
            });
            li.appendChild(a);
            tocListEl.appendChild(li);
        });
    }

    // Pagination (Prev / Next)
    function updatePagination(docId) {
        const index = DOCS_CATALOG.findIndex(d => d.id === docId);
        const prevDoc = DOCS_CATALOG[index - 1];
        const nextDoc = DOCS_CATALOG[index + 1];

        if (prevDoc) {
            paginationPrevEl.style.display = 'flex';
            paginationPrevEl.querySelector('.pagination-title').textContent = prevDoc.title;
            paginationPrevEl.onclick = (e) => { e.preventDefault(); loadDocument(prevDoc.id); };
        } else {
            paginationPrevEl.style.display = 'none';
        }

        if (nextDoc) {
            paginationNextEl.style.display = 'flex';
            paginationNextEl.querySelector('.pagination-title').textContent = nextDoc.title;
            paginationNextEl.onclick = (e) => { e.preventDefault(); loadDocument(nextDoc.id); };
        } else {
            paginationNextEl.style.display = 'none';
        }
    }

    // Copy Snippet Action
    window.copySnippet = function (btn) {
        const code = btn.getAttribute('data-code');
        if (!code) return;

        navigator.clipboard.writeText(code).then(() => {
            btn.classList.add('copied');
            const span = btn.querySelector('span');
            const origText = span.textContent;
            span.textContent = 'Copied!';
            setTimeout(() => {
                btn.classList.remove('copied');
                span.textContent = origText;
            }, 2000);
        });
    };

    // Event Listeners
    searchInput.addEventListener('input', (e) => {
        renderDocsList(e.target.value);
    });

    mobileToggleBtn.addEventListener('click', () => {
        sidebarEl.classList.toggle('open');
    });

    window.addEventListener('popstate', () => {
        const hash = window.location.hash.slice(1);
        if (hash) {
            loadDocument(hash);
        }
    });

    // Initial Launch
    const initialHash = window.location.hash.slice(1);
    const initialDoc = DOCS_CATALOG.find(d => d.id === initialHash);
    renderDocsList();
    loadDocument(initialDoc ? initialDoc.id : 'getting-started');

})();
