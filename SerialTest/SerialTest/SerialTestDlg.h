
// SerialTestDlg.h : 头文件
//

#pragma once
#include "afxwin.h"
#include "mscomm1.h"


// CSerialTestDlg 对话框
class CSerialTestDlg : public CDialogEx
{
// 构造
public:
	CSerialTestDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SERIALTEST_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_comboCom;
	afx_msg void OnCbnSelchangeComboCom();
	CMscomm1 m_ctrlComm;
private:
	bool m_setOk;
public:
	CString m_strTXData;
	CString m_strRXData;
	DECLARE_EVENTSINK_MAP()
	void OnComm();
	afx_msg void OnBnClickedButtonSend();
	afx_msg void OnBnClickedButtonCleanup();
	CComboBox m_combobox_bound;
};
