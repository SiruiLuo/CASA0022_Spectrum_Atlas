// 导航栏交互
document.addEventListener('DOMContentLoaded', function() {
    const hamburger = document.querySelector('.hamburger');
    const navMenu = document.querySelector('.nav-menu');
    
    hamburger.addEventListener('click', () => {
        hamburger.classList.toggle('active');
        navMenu.classList.toggle('active');
    });
    
    // 平滑滚动
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            e.preventDefault();
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                target.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        });
    });
    
    // 滚动动画
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };
    
    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('visible');
            }
        });
    }, observerOptions);
    
    document.querySelectorAll('.tech-card, .heatmap-item, .stat-item').forEach(el => {
        el.classList.add('fade-in');
        observer.observe(el);
    });
    
    // 热力图演示动画
    const heatmapGrid = document.querySelector('.heatmap-grid');
    if (heatmapGrid) {
        for (let i = 0; i < 400; i++) {
            const cell = document.createElement('div');
            cell.style.background = `rgba(0, 0, 0, ${Math.random() * 0.3})`;
            cell.style.animationDelay = `${Math.random() * 3}s`;
            cell.style.animationDuration = `${2 + Math.random() * 2}s`;
            heatmapGrid.appendChild(cell);
        }
    }
    
    // 表单验证
    const contactForm = document.querySelector('.contact-form form');
    if (contactForm) {
        contactForm.addEventListener('submit', function(e) {
            e.preventDefault();
            
            const name = this.querySelector('input[type="text"]').value;
            const email = this.querySelector('input[type="email"]').value;
            const message = this.querySelector('textarea').value;
            
            if (name && email && message) {
                showNotification('Message sent successfully!', 'success');
                this.reset();
            } else {
                showNotification('Please fill in all fields.', 'error');
            }
        });
    }
    
    // 热力图模态框功能
    const modal = document.getElementById('heatmapModal');
    const modalImage = document.getElementById('modalImage');
    const modalTitle = document.getElementById('modalTitle');
    const modalDescription = document.getElementById('modalDescription');
    const closeBtn = document.querySelector('.close');
    
    // 点击热力图打开模态框
    document.querySelectorAll('.heatmap-item').forEach(item => {
        item.addEventListener('click', function() {
            const heatmapSrc = this.getAttribute('data-heatmap');
            const title = this.getAttribute('data-title');
            const description = this.getAttribute('data-description');
            
            modalImage.src = heatmapSrc;
            modalImage.alt = title;
            modalTitle.textContent = title;
            modalDescription.textContent = description;
            
            modal.style.display = 'block';
            document.body.style.overflow = 'hidden'; // 防止背景滚动
        });
    });
    
    // 点击关闭按钮关闭模态框
    closeBtn.addEventListener('click', closeModal);
    
    // 点击模态框背景关闭模态框
    modal.addEventListener('click', function(e) {
        if (e.target === modal) {
            closeModal();
        }
    });
    
    // 按ESC键关闭模态框
    document.addEventListener('keydown', function(e) {
        if (e.key === 'Escape' && modal.style.display === 'block') {
            closeModal();
        }
    });
    
    function closeModal() {
        modal.style.display = 'none';
        document.body.style.overflow = 'auto'; // 恢复背景滚动
    }
    
    // 通知功能
    function showNotification(message, type = 'info') {
        const notification = document.createElement('div');
        notification.className = `notification ${type}`;
        notification.textContent = message;
        
        // 添加样式
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 15px 20px;
            border-radius: 8px;
            color: white;
            font-weight: 500;
            z-index: 10001;
            transform: translateX(100%);
            transition: transform 0.3s ease;
            max-width: 300px;
        `;
        
        // 根据类型设置背景色
        switch(type) {
            case 'success':
                notification.style.background = '#4CAF50';
                break;
            case 'error':
                notification.style.background = '#f44336';
                break;
            default:
                notification.style.background = '#2196F3';
        }
        
        document.body.appendChild(notification);
        
        // 显示动画
        setTimeout(() => {
            notification.style.transform = 'translateX(0)';
        }, 100);
        
        // 自动隐藏
        setTimeout(() => {
            notification.style.transform = 'translateX(100%)';
            setTimeout(() => {
                document.body.removeChild(notification);
            }, 300);
        }, 3000);
    }
    
    // 导航栏滚动效果
    let lastScrollTop = 0;
    window.addEventListener('scroll', function() {
        const navbar = document.querySelector('.navbar');
        const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
        
        if (scrollTop > lastScrollTop && scrollTop > 100) {
            // 向下滚动
            navbar.style.transform = 'translateY(-100%)';
        } else {
            // 向上滚动
            navbar.style.transform = 'translateY(0)';
        }
        
        lastScrollTop = scrollTop;
    });
    
    // 热力图悬停效果增强
    document.querySelectorAll('.heatmap-item').forEach(item => {
        item.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-10px) scale(1.02)';
        });
        
        item.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0) scale(1)';
        });
    });
    
    // 技术卡片悬停效果
    document.querySelectorAll('.tech-card').forEach(card => {
        card.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-8px)';
            this.style.boxShadow = '0 8px 30px rgba(0, 0, 0, 0.2)';
        });
        
        card.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0)';
            this.style.boxShadow = '0 4px 20px rgba(0, 0, 0, 0.15)';
        });
    });
    
    // 按钮悬停效果
    document.querySelectorAll('.btn').forEach(btn => {
        btn.addEventListener('mouseenter', function() {
            this.style.transform = 'translateY(-3px)';
        });
        
        btn.addEventListener('mouseleave', function() {
            this.style.transform = 'translateY(0)';
        });
    });
    
    // 页面加载动画
    window.addEventListener('load', function() {
        document.body.style.opacity = '0';
        document.body.style.transition = 'opacity 0.5s ease';
        
        setTimeout(() => {
            document.body.style.opacity = '1';
        }, 100);
    });
    
    // 滚动进度指示器
    const progressBar = document.createElement('div');
    progressBar.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 0%;
        height: 3px;
        background: linear-gradient(90deg, #000000, #666666);
        z-index: 10002;
        transition: width 0.1s ease;
    `;
    document.body.appendChild(progressBar);
    
    window.addEventListener('scroll', function() {
        const scrollTop = window.pageYOffset || document.documentElement.scrollTop;
        const scrollHeight = document.documentElement.scrollHeight - window.innerHeight;
        const scrollProgress = (scrollTop / scrollHeight) * 100;
        progressBar.style.width = scrollProgress + '%';
    });
    
    // 键盘导航支持
    document.addEventListener('keydown', function(e) {
        // 在模态框打开时，Tab键应该循环在模态框内的元素
        if (modal.style.display === 'block') {
            const focusableElements = modal.querySelectorAll('button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])');
            const firstElement = focusableElements[0];
            const lastElement = focusableElements[focusableElements.length - 1];
            
            if (e.key === 'Tab') {
                if (e.shiftKey) {
                    if (document.activeElement === firstElement) {
                        e.preventDefault();
                        lastElement.focus();
                    }
                } else {
                    if (document.activeElement === lastElement) {
                        e.preventDefault();
                        firstElement.focus();
                    }
                }
            }
        }
    });
    
    // 热力图加载优化
    document.querySelectorAll('.heatmap-image img').forEach(img => {
        img.addEventListener('load', function() {
            this.style.opacity = '1';
        });
        
        img.addEventListener('error', function() {
            this.style.opacity = '0.5';
            this.style.filter = 'grayscale(100%)';
        });
    });
    
    // 响应式导航菜单
    const mediaQuery = window.matchMedia('(max-width: 768px)');
    
    function handleMobileMenu(e) {
        if (e.matches) {
            // 移动端菜单
            navMenu.style.position = 'fixed';
            navMenu.style.top = '70px';
            navMenu.style.left = '-100%';
            navMenu.style.width = '100%';
            navMenu.style.height = 'calc(100vh - 70px)';
            navMenu.style.background = 'rgba(255, 255, 255, 0.95)';
            navMenu.style.backdropFilter = 'blur(10px)';
            navMenu.style.flexDirection = 'column';
            navMenu.style.justifyContent = 'flex-start';
            navMenu.style.paddingTop = '2rem';
            navMenu.style.transition = 'left 0.3s ease';
        } else {
            // 桌面端菜单
            navMenu.style.position = 'static';
            navMenu.style.top = 'auto';
            navMenu.style.left = 'auto';
            navMenu.style.width = 'auto';
            navMenu.style.height = 'auto';
            navMenu.style.background = 'transparent';
            navMenu.style.backdropFilter = 'none';
            navMenu.style.flexDirection = 'row';
            navMenu.style.justifyContent = 'flex-end';
            navMenu.style.paddingTop = '0';
        }
    }
    
    mediaQuery.addListener(handleMobileMenu);
    handleMobileMenu(mediaQuery);
    
    // 移动端菜单切换
    hamburger.addEventListener('click', function() {
        if (navMenu.style.left === '0px' || navMenu.style.left === '') {
            navMenu.style.left = '-100%';
        } else {
            navMenu.style.left = '0px';
        }
    });
    
    // 点击菜单项后关闭移动端菜单
    navMenu.querySelectorAll('a').forEach(link => {
        link.addEventListener('click', function() {
            if (mediaQuery.matches) {
                navMenu.style.left = '-100%';
            }
        });
    });
}); 