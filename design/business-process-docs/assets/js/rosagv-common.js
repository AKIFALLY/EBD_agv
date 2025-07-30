/**
 * RosAGV 統一 JavaScript 功能庫
 * 提供導航、動畫、互動功能
 */

// 全域配置
const RosAGV = {
    config: {
        animationDuration: 600,
        scrollOffset: 80,
        autoScrollSpeed: 50
    },
    
    // 初始化函數
    init: function() {
        this.setupNavigation();
        this.setupScrollEffects();
        this.setupAnimations();
        this.setupInteractiveElements();
        this.setupTheme();
        console.log('RosAGV JS Library Initialized');
    },

    // 導航功能
    setupNavigation: function() {
        const navLinks = document.querySelectorAll('.nav-links a');
        const sections = document.querySelectorAll('.section');

        // 平滑滾動
        navLinks.forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                const targetId = link.getAttribute('href').substring(1);
                const targetSection = document.getElementById(targetId);
                
                if (targetSection) {
                    this.smoothScrollTo(targetSection, this.config.scrollOffset);
                }
            });
        });

        // 滾動時高亮當前導航
        window.addEventListener('scroll', () => {
            this.updateActiveNavigation(sections, navLinks);
        });
    },

    // 平滑滾動功能
    smoothScrollTo: function(element, offset = 0) {
        const elementPosition = element.offsetTop - offset;
        const startPosition = window.pageYOffset;
        const distance = elementPosition - startPosition;
        const duration = this.config.animationDuration;
        let start = null;

        const animation = (currentTime) => {
            if (start === null) start = currentTime;
            const timeElapsed = currentTime - start;
            const run = this.easeInOutQuad(timeElapsed, startPosition, distance, duration);
            window.scrollTo(0, run);
            if (timeElapsed < duration) requestAnimationFrame(animation);
        };

        requestAnimationFrame(animation);
    },

    // 緩動函數
    easeInOutQuad: function(t, b, c, d) {
        t /= d / 2;
        if (t < 1) return c / 2 * t * t + b;
        t--;
        return -c / 2 * (t * (t - 2) - 1) + b;
    },

    // 更新活躍導航
    updateActiveNavigation: function(sections, navLinks) {
        const scrollPosition = window.scrollY + this.config.scrollOffset;

        sections.forEach((section, index) => {
            const sectionTop = section.offsetTop;
            const sectionHeight = section.offsetHeight;
            
            if (scrollPosition >= sectionTop && scrollPosition < sectionTop + sectionHeight) {
                navLinks.forEach(link => link.classList.remove('active'));
                if (navLinks[index]) {
                    navLinks[index].classList.add('active');
                }
            }
        });
    },

    // 滾動效果
    setupScrollEffects: function() {
        const observerOptions = {
            threshold: 0.1,
            rootMargin: '0px 0px -50px 0px'
        };

        const observer = new IntersectionObserver((entries) => {
            entries.forEach(entry => {
                if (entry.isIntersecting) {
                    entry.target.classList.add('fade-in-up');
                }
            });
        }, observerOptions);

        // 觀察所有需要動畫的元素
        const animatedElements = document.querySelectorAll('.section, .feature-item, .stat-item, .card, .timeline-item');
        animatedElements.forEach(element => {
            observer.observe(element);
        });
    },

    // 設置動畫
    setupAnimations: function() {
        // 數字計數動畫
        const statNumbers = document.querySelectorAll('.stat-number');
        statNumbers.forEach(number => {
            const targetValue = parseInt(number.textContent);
            this.animateNumber(number, 0, targetValue, 2000);
        });

        // 進度條動畫
        const progressBars = document.querySelectorAll('.progress-bar');
        progressBars.forEach(bar => {
            const targetWidth = bar.getAttribute('data-width') || '0%';
            setTimeout(() => {
                bar.style.width = targetWidth;
            }, 500);
        });
    },

    // 數字動畫
    animateNumber: function(element, start, end, duration) {
        const range = end - start;
        const increment = range / (duration / 16); // 60fps
        let current = start;
        
        const timer = setInterval(() => {
            current += increment;
            if (current >= end) {
                current = end;
                clearInterval(timer);
            }
            element.textContent = Math.floor(current).toLocaleString();
        }, 16);
    },

    // 互動元素設置
    setupInteractiveElements: function() {
        // 卡片懸停效果
        const cards = document.querySelectorAll('.card, .feature-item, .stat-item');
        cards.forEach(card => {
            card.addEventListener('mouseenter', () => {
                card.style.transform = 'translateY(-8px)';
            });
            
            card.addEventListener('mouseleave', () => {
                card.style.transform = 'translateY(0)';
            });
        });

        // 按鈕點擊效果
        const buttons = document.querySelectorAll('.btn');
        buttons.forEach(button => {
            button.addEventListener('click', (e) => {
                this.createRippleEffect(e, button);
            });
        });

        // 表格行懸停效果
        const tableRows = document.querySelectorAll('tbody tr');
        tableRows.forEach(row => {
            row.addEventListener('mouseenter', () => {
                row.style.backgroundColor = '#f8f9fa';
            });
            
            row.addEventListener('mouseleave', () => {
                row.style.backgroundColor = '';
            });
        });
    },

    // 波紋效果
    createRippleEffect: function(event, element) {
        const ripple = document.createElement('span');
        const rect = element.getBoundingClientRect();
        const size = Math.max(rect.width, rect.height);
        const x = event.clientX - rect.left - size / 2;
        const y = event.clientY - rect.top - size / 2;
        
        ripple.style.cssText = `
            position: absolute;
            width: ${size}px;
            height: ${size}px;
            left: ${x}px;
            top: ${y}px;
            background: rgba(255, 255, 255, 0.3);
            border-radius: 50%;
            transform: scale(0);
            animation: ripple 0.6s linear;
            pointer-events: none;
        `;
        
        element.style.position = 'relative';
        element.style.overflow = 'hidden';
        element.appendChild(ripple);
        
        setTimeout(() => {
            ripple.remove();
        }, 600);
    },

    // 主題設置
    setupTheme: function() {
        // 從 URL 或本地存儲獲取主題
        const urlParams = new URLSearchParams(window.location.search);
        const themeFromUrl = urlParams.get('theme');
        const savedTheme = localStorage.getItem('rosagv-theme');
        const theme = themeFromUrl || savedTheme || 'blue';
        
        this.applyTheme(theme);
        
        // 主題切換按鈕
        const themeButtons = document.querySelectorAll('[data-theme]');
        themeButtons.forEach(button => {
            button.addEventListener('click', (e) => {
                const newTheme = e.target.getAttribute('data-theme');
                this.applyTheme(newTheme);
                localStorage.setItem('rosagv-theme', newTheme);
            });
        });
    },

    // 應用主題
    applyTheme: function(theme) {
        const body = document.body;
        const themes = ['blue', 'green', 'orange', 'purple'];
        
        // 移除現有主題類
        themes.forEach(t => {
            body.classList.remove(`theme-${t}`);
        });
        
        // 添加新主題類
        body.classList.add(`theme-${theme}`);
        
        // 更新 CSS 變數
        const root = document.documentElement;
        const themeColors = {
            blue: '#3498db',
            green: '#27ae60',
            orange: '#e67e22',
            purple: '#9b59b6'
        };
        
        root.style.setProperty('--theme-color', themeColors[theme]);
    },

    // 工具函數
    utils: {
        // 防抖函數
        debounce: function(func, wait) {
            let timeout;
            return function executedFunction(...args) {
                const later = () => {
                    clearTimeout(timeout);
                    func(...args);
                };
                clearTimeout(timeout);
                timeout = setTimeout(later, wait);
            };
        },

        // 節流函數
        throttle: function(func, limit) {
            let inThrottle;
            return function() {
                const args = arguments;
                const context = this;
                if (!inThrottle) {
                    func.apply(context, args);
                    inThrottle = true;
                    setTimeout(() => inThrottle = false, limit);
                }
            };
        },

        // 格式化數字
        formatNumber: function(num) {
            return num.toLocaleString();
        },

        // 截取文字
        truncateText: function(text, length) {
            return text.length > length ? text.substring(0, length) + '...' : text;
        },

        // 檢測移動設備
        isMobile: function() {
            return window.innerWidth <= 768;
        },

        // 顯示載入狀態
        showLoading: function(element) {
            element.innerHTML = '<span class="loading"></span> 載入中...';
            element.disabled = true;
        },

        // 隱藏載入狀態
        hideLoading: function(element, originalText) {
            element.innerHTML = originalText;
            element.disabled = false;
        }
    },

    // 頁面特定功能
    page: {
        // 統計數據更新
        updateStats: function(statsData) {
            Object.keys(statsData).forEach(key => {
                const element = document.querySelector(`[data-stat="${key}"]`);
                if (element) {
                    RosAGV.animateNumber(element, 0, statsData[key], 1500);
                }
            });
        },

        // 動態載入內容
        loadContent: function(url, containerId) {
            const container = document.getElementById(containerId);
            if (!container) return;

            RosAGV.utils.showLoading(container);

            fetch(url)
                .then(response => response.text())
                .then(html => {
                    container.innerHTML = html;
                    RosAGV.setupAnimations();
                })
                .catch(error => {
                    console.error('載入內容失敗:', error);
                    container.innerHTML = '<p>載入失敗，請重試。</p>';
                });
        },

        // 導出功能
        exportToPDF: function() {
            if (window.html2pdf) {
                const element = document.body;
                const opt = {
                    margin: 1,
                    filename: `rosagv-document-${new Date().toISOString().split('T')[0]}.pdf`,
                    image: { type: 'jpeg', quality: 0.98 },
                    html2canvas: { scale: 2 },
                    jsPDF: { unit: 'in', format: 'letter', orientation: 'portrait' }
                };
                html2pdf().set(opt).from(element).save();
            } else {
                alert('PDF 導出功能需要額外的庫支援');
            }
        }
    }
};

// CSS 動畫樣式注入
const animationCSS = `
    @keyframes ripple {
        to {
            transform: scale(4);
            opacity: 0;
        }
    }
`;

// 注入動畫樣式
const style = document.createElement('style');
style.textContent = animationCSS;
document.head.appendChild(style);

// DOM 載入完成後初始化
document.addEventListener('DOMContentLoaded', () => {
    RosAGV.init();
});

// 視窗調整大小時的處理
window.addEventListener('resize', RosAGV.utils.debounce(() => {
    // 重新計算動畫和佈局
    console.log('視窗大小已調整');
}, 250));

// 導出全域對象
window.RosAGV = RosAGV;