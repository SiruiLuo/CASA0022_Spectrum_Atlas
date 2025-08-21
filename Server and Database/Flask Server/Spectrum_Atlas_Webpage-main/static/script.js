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
        // 清空现有内容
        heatmapGrid.innerHTML = '';
        
        // 创建400个网格单元
        for (let i = 0; i < 400; i++) {
            const cell = document.createElement('div');
            cell.style.cssText = `
                background: rgba(0, 0, 0, ${Math.random() * 0.3});
                animation: heatmapCellPulse ${2 + Math.random() * 3}s infinite;
                animation-delay: ${Math.random() * 2}s;
                transition: all 0.3s ease;
            `;
            heatmapGrid.appendChild(cell);
        }
        
        // 添加动态数据更新模拟
        setInterval(() => {
            const cells = heatmapGrid.querySelectorAll('div');
            cells.forEach(cell => {
                const newOpacity = Math.random() * 0.4;
                const newColor = `rgba(0, 0, 0, ${newOpacity})`;
                cell.style.background = newColor;
            });
        }, 2000);
        
        // 添加鼠标交互效果
        heatmapGrid.addEventListener('mousemove', (e) => {
            const rect = heatmapGrid.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            
            const cells = heatmapGrid.querySelectorAll('div');
            cells.forEach((cell, index) => {
                const cellRect = cell.getBoundingClientRect();
                const cellX = cellRect.left - rect.left + cellRect.width / 2;
                const cellY = cellRect.top - rect.top + cellRect.height / 2;
                
                const distance = Math.sqrt((x - cellX) ** 2 + (y - cellY) ** 2);
                const maxDistance = 100;
                
                if (distance < maxDistance) {
                    const intensity = 1 - (distance / maxDistance);
                    cell.style.background = `rgba(0, 0, 0, ${0.1 + intensity * 0.4})`;
                    cell.style.transform = `scale(${1 + intensity * 0.1})`;
                } else {
                    cell.style.transform = 'scale(1)';
                }
            });
        });
        
        // 鼠标离开时恢复
        heatmapGrid.addEventListener('mouseleave', () => {
            const cells = heatmapGrid.querySelectorAll('div');
            cells.forEach(cell => {
                cell.style.transform = 'scale(1)';
            });
        });
        
        // 点击热力图时的交互效果
        heatmapGrid.addEventListener('click', (e) => {
            const rect = heatmapGrid.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            
            // 创建点击波纹效果
            const ripple = document.createElement('div');
            ripple.style.cssText = `
                position: absolute;
                left: ${x}px;
                top: ${y}px;
                width: 0;
                height: 0;
                border-radius: 50%;
                background: rgba(102, 102, 102, 0.3);
                transform: translate(-50%, -50%);
                animation: rippleExpand 0.6s ease-out;
                pointer-events: none;
                z-index: 10;
            `;
            
            heatmapGrid.appendChild(ripple);
            
            // 移除波纹元素
            setTimeout(() => {
                if (ripple.parentNode) {
                    ripple.parentNode.removeChild(ripple);
                }
            }, 600);
            
            // 显示点击位置的数据强度
            const intensity = Math.random() * 100;
            showNotification(`Signal strength at this point: ${intensity.toFixed(1)} dBm`, 'info');
        });
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
    window.showNotification = function(message, type = 'info') {
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

    // 实时数据统计更新
    const statNumbers = document.querySelectorAll('.stat-number');
    if (statNumbers.length > 0) {
        // 处理速度主题列表
        const processingThemes = [
            'Instant',
            'Live',
            'Active',
            'Dynamic',
            'Fast',
            'Quick',
            'Rapid'
        ];
        
        let themeIndex = 0;
        
        // 只更新Processing Speed，其他保持不变
        setInterval(() => {
            statNumbers.forEach((stat, index) => {
                const currentText = stat.textContent;
                
                // 只更新第三个统计项（Processing Speed）
                if (index === 2) {
                    stat.textContent = processingThemes[themeIndex];
                    themeIndex = (themeIndex + 1) % processingThemes.length;
                }
                // Data Accuracy (index 0) 和 Frequency Bands (index 1) 保持不变
            });
        }, 2000);
        
        // 添加数字变化动画
        statNumbers.forEach(stat => {
            stat.style.transition = 'all 0.5s ease';
        });
    }
    
    // 信号强度模拟
    const signalWaves = document.querySelectorAll('.signal-wave');
    if (signalWaves.length > 0) {
        setInterval(() => {
            signalWaves.forEach((wave, index) => {
                const intensity = Math.random();
                wave.style.borderColor = `rgba(102, 102, 102, ${intensity})`;
                wave.style.animationDuration = `${1.5 + Math.random()}s`;
            });
        }, 1000);
    }
    
    // 状态指示器动态更新
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.status-text');
    if (statusDot && statusText) {
        const statuses = [
            { text: 'System Active', color: '#4CAF50' },
            { text: 'Collecting Data', color: '#2196F3' },
            { text: 'Processing Signals', color: '#FF9800' },
            { text: 'Generating Heatmap', color: '#9C27B0' }
        ];
        
        let currentStatus = 0;
        
        setInterval(() => {
            const status = statuses[currentStatus];
            statusDot.style.background = status.color;
            statusText.textContent = status.text;
            
            currentStatus = (currentStatus + 1) % statuses.length;
        }, 3000);
    }

    // 交互式热力图功能
    initializeInteractiveHeatmap();
});

// 交互式热力图功能
function initializeInteractiveHeatmap() {
    const signalTypeSelect = document.getElementById('signal-type-select');
    const sessionSelect = document.getElementById('session-select');
    const generateBtn = document.getElementById('generate-heatmap-btn');
    const topViewBtn = document.getElementById('top-view-btn'); // 新增
    let heatmap3D = null;
    let currentHeatmapData = null;
    
    // 初始化3D热力图渲染器
    function init3DHeatmap() {
        const container = document.getElementById('heatmap-3d-container');
        if (container && !heatmap3D) {
            heatmap3D = new Heatmap3DRenderer('heatmap-3d-container');
        }
    }
    
    // 页面加载完成后初始化3D渲染器
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init3DHeatmap);
    } else {
        init3DHeatmap();
    }
    
    // 加载信号类型
    loadSignalTypes();
    
    // 加载会话数据
    loadSessions();
    
    // 监听选择变化
    signalTypeSelect.addEventListener('change', updateGenerateButton);
    sessionSelect.addEventListener('change', updateGenerateButton);
    
    // 生成热力图按钮
    generateBtn.addEventListener('click', generateHeatmap);
    // 新增：俯视视角按钮事件
    if (topViewBtn) {
        topViewBtn.addEventListener('click', ()=>{
            if (heatmap3D && typeof heatmap3D.setTopView === 'function') {
                heatmap3D.setTopView();
            }
        });
    }
    
    // 3D渲染器会自动处理鼠标事件
    
    function loadSignalTypes() {
        fetch('/api/signal_types')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Failed to load signal types');
                }
                return response.json();
            })
            .then(signalTypes => {
                console.log('Available signal types:', signalTypes);
                
                // 清空现有选项
                signalTypeSelect.innerHTML = '<option value="">Select a signal type...</option>';
                
                // 添加可用的信号类型
                signalTypes.forEach(signalType => {
                    const option = document.createElement('option');
                    option.value = signalType;
                    option.textContent = signalType;
                    signalTypeSelect.appendChild(option);
                });
                
                // 更新生成按钮状态
                updateGenerateButton();
            })
            .catch(error => {
                console.error('Error loading signal types:', error);
                showNotification('Failed to load signal types', 'error');
            });
    }

    function loadSessions() {
        fetch('/api/sessions')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Failed to load sessions');
                }
                return response.json();
            })
            .then(sessions => {
                console.log('Available sessions:', sessions);
                
                // 清空现有选项
                sessionSelect.innerHTML = '<option value="">Select a session...</option>';
                
                // 添加可用的会话
                sessions.forEach(session => {
                    const option = document.createElement('option');
                    option.value = session.id;
                    option.textContent = session.id;
                    sessionSelect.appendChild(option);
                });
                
                // 更新生成按钮状态
                updateGenerateButton();
            })
            .catch(error => {
                console.error('Error loading sessions:', error);
                showNotification('Failed to load sessions', 'error');
            });
    }
    
    function updateGenerateButton() {
        const signalType = signalTypeSelect.value;
        const session = sessionSelect.value;
        generateBtn.disabled = !signalType || !session;
    }
    
    function generateHeatmap() {
        const signalType = signalTypeSelect.value;
        const session = sessionSelect.value;
        
        if (!signalType || !session) {
            showNotification('Please select both signal type and session', 'error');
            return;
        }
        
        // 显示加载状态
        showLoadingState();
        
        fetch(`/api/heatmap/${encodeURIComponent(signalType)}/${session}`)
            .then(response => {
                if (!response.ok) {
                    // 尝试解析错误响应
                    return response.json().then(errorData => {
                        throw new Error(errorData.error || `HTTP ${response.status}: ${response.statusText}`);
                    }).catch(() => {
                        // 如果无法解析JSON，使用默认错误消息
                        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
                    });
                }
                return response.json();
            })
            .then(data => {
                console.log('Received heatmap data:', data);
                currentHeatmapData = data;
                // 先隐藏加载状态，确保canvas被正确创建
                hideLoadingState();
                // 然后渲染热力图
                renderHeatmap(data);
                updateInfoPanel(data);
                showNotification('Heatmap generated successfully', 'success');
            })
            .catch(error => {
                console.error('Error generating heatmap:', error);
                hideLoadingState();
                
                // 根据错误类型显示不同的消息
                let errorMessage = error.message || 'Failed to generate heatmap';
                
                if (error.message.includes('404') || error.message.includes('No data found')) {
                    errorMessage = `No data available for "${signalType}" in session "${session}". Please try a different signal type or session.`;
                } else if (error.message.includes('500')) {
                    errorMessage = 'Server error occurred. Please try again later.';
                } else if (error.message.includes('Failed to fetch')) {
                    errorMessage = 'Network error. Please check your connection and try again.';
                }
                
                showNotification(errorMessage, 'error');
            });
    }
    
    function showLoadingState() {
        const container = document.getElementById('heatmap-canvas-container');
        container.innerHTML = '<div class="loading">Generating heatmap...</div>';
    }
    
    function hideLoadingState() {
        const container = document.getElementById('heatmap-canvas-container');
        if (!container) {
            console.error('Heatmap container not found!');
            return;
        }
        
        // 清空容器并重新创建3D容器
        container.innerHTML = '';
        
        // 创建新的3D容器
        const new3DContainer = document.createElement('div');
        new3DContainer.id = 'heatmap-3d-container';
        
        // 创建新的tooltip元素
        const newTooltip = document.createElement('div');
        newTooltip.id = 'heatmap-tooltip';
        newTooltip.className = 'heatmap-tooltip';

        // —— 创建新的俯视按钮 ——
        const newTopBtn = document.createElement('button');
        newTopBtn.id = 'top-view-btn';
        newTopBtn.className = 'top-view-btn';
        newTopBtn.title = 'Top View';
        newTopBtn.innerHTML = '<i class="fas fa-eye"></i>';

        // 将元素添加到容器
        container.appendChild(new3DContainer);
        container.appendChild(newTopBtn);
        document.body.appendChild(newTooltip);

        // 绑定点击事件
        newTopBtn.addEventListener('click', ()=>{
            if (heatmap3D && typeof heatmap3D.setTopView === 'function') {
                heatmap3D.setTopView();
            }
        });
        
        // 重新初始化3D渲染器
        if (heatmap3D) {
            heatmap3D.dispose();
        }
        heatmap3D = new Heatmap3DRenderer('heatmap-3d-container');
        
        console.log('3D container reinitialized:', new3DContainer);
        console.log('3D container parent after reinit:', new3DContainer.parentElement);
    }
    
    function renderHeatmap(data) {
        console.log('Rendering 3D heatmap with data:', data);
        
        if (!heatmap3D) {
            console.error('3D heatmap renderer not initialized');
            return;
        }
        
        // 设置当前信号类型，便于推算网速
        if (heatmap3D.setSignalType) {
            heatmap3D.setSignalType(signalTypeSelect.value);
        }

        // 使用3D渲染器渲染热力图
        heatmap3D.renderHeatmap(data);
        
        // 更新信息面板
        updateInfoPanel(data);
    }
    
    // HSL 转 RGB 辅助函数（h,s,l 均为 0-1 区间），返回 [r,g,b]（0-1 区间）
    function hslToRgb(h, s, l) {
        let r, g, b;
        if (s === 0) {
            r = g = b = l; // achromatic
        } else {
            const hue2rgb = (p, q, t) => {
                if (t < 0) t += 1;
                if (t > 1) t -= 1;
                if (t < 1 / 6) return p + (q - p) * 6 * t;
                if (t < 1 / 2) return q;
                if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
                return p;
            };
            const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
            const p = 2 * l - q;
            r = hue2rgb(p, q, h + 1 / 3);
            g = hue2rgb(p, q, h);
            b = hue2rgb(p, q, h - 1 / 3);
        }
        return [r, g, b];
    }

    // 根据 RSSI 值映射为彩虹色：红(强)→紫(弱)
    function getHeatmapColor(value, min, max) {
        const normalized = (value - min) / (max - min); // 0(弱)→1(强)
        const clamped = Math.max(0, Math.min(1, normalized));
        // Hue 从 0°(红) 到 270°(紫)。强→红，弱→紫，所以 hue = (1 - clamped) * 270°
        const hueDeg = (1 - clamped) * 270;
        const [rN, gN, bN] = hslToRgb(hueDeg / 360, 1, 0.5);
        return {
            r: Math.round(rN * 255),
            g: Math.round(gN * 255),
            b: Math.round(bN * 255)
        };
    }
    

    
    function updateInfoPanel(data) {
        document.getElementById('current-signal-type').textContent = signalTypeSelect.options[signalTypeSelect.selectedIndex].text;
        document.getElementById('current-session').textContent = sessionSelect.options[sessionSelect.selectedIndex].text;
        document.getElementById('min-rssi').textContent = `${data.min_rssi.toFixed(1)} dBm`;
        document.getElementById('max-rssi').textContent = `${data.max_rssi.toFixed(1)} dBm`;
        document.getElementById('data-points').textContent = data.sample_points.x.length;
        // 计算理论速率
        const BW_MHZ_MAP = {
            'FM Radio': 0.2,
            'ADS-B 1090': 1,
            'Airband (AM)': 0.025,
            'AIS / Marine': 0.025,
            'GSM 900 DL': 0.2,
            'GSM 900 UL': 0.2,
            'ISM 868': 0.125,
            'LoRa / ISM 433': 0.125,
            'LTE 1800': 20,
            'RC Aircraft': 0.05,
            'RC Ground': 0.05,
            'RC Low Band': 0.05,
            'TETRA / Emergency': 0.025
        };

        const noiseFloor = -100;
        const bw = (BW_MHZ_MAP[signalTypeSelect.value] || 1) * 1e6;
        const rssiToMbps = rssi=>{
            const snrDb = rssi - noiseFloor;
            const snrLin = Math.pow(10, snrDb/10);
            const capacity = bw * Math.log2(1+snrLin);
            return (capacity/1e6).toFixed(2);
        };
        const minSpeed = rssiToMbps(data.min_rssi);
        const maxSpeed = rssiToMbps(data.max_rssi);
        document.getElementById('min-speed').textContent = `${minSpeed} Mbps`;
        document.getElementById('max-speed').textContent = `${maxSpeed} Mbps`;
    }
} 